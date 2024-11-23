// Copyright (c) 2019 ETH Zurich, University of Bologna
//
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
//
// Authors:
// - Simone Manoni <s.manoni@unibo.it>

/// Simulation-Only injector for AXI transactions
///
/// This module writes all handshaked AXI beats to a log file. To use in simulation,
/// ensure `TARGET_SIMULATION` is defined.

module axi_injector #(
  parameter      FileName   = "axi_bus",
  parameter bit  InjAW      = 1'b1,
  parameter bit  InjAR      = 1'b1,
  parameter bit  InjW       = 1'b0,
  parameter bit  InjR       = 1'b0,
  parameter      AxiDataWidth = 10'd512,
  parameter type axi_req_t  = logic,
  parameter type axi_resp_t = logic
) (
  input  logic clk_i,
  input  logic rst_ni,
  output axi_req_t axi_req_o,
  input  axi_resp_t axi_resp_i
);

  // Internal transaction structure
  typedef struct {
    int    timestep;
    string line_data;
  } transaction_t;

  // Define states using an enumerated type for better readability
  typedef enum logic [1:0] {
    IDLE,         
    ISSUE
  } state_t;

  state_t current_state, next_state; // State variables
  int quque_size; 

  transaction_t transaction_queue[$:4]; // Queue of transactions

  axi_req_t      current_req;  // Holds the current request being emitted
  string         req_type;
  transaction_t  transaction_data;
  transaction_t transaction;
  logic pending_transaction;
  logic emitted_trans;
  logic enable_trans;
  logic debug;
  logic fifo_empty;
  int sim_time;

  initial begin
    string file_line;
    int file;
    file = $fopen(FileName, "r");
    if (file == 0) begin
      $fatal(1, "Failed to open file: %s", FileName);
    end

    while (!$feof(file)) begin
      if (transaction_queue.size() <= 4) begin 
        file_line = "";
        $fgets(file_line, file);  
        transaction.timestep = extract_value(file_line, "time");
        transaction.line_data = file_line;
        transaction_queue.push_back(transaction);
        transaction_data.timestep = transaction_queue[0].timestep;
        transaction_data.line_data = transaction_queue[0].line_data;
      end 
      //$display("tran queue %d",transaction_data.timestep);
      @(posedge clk_i);
    end  
  end

  always @(posedge clk_i) begin
    sim_time = $time;
  end 

  always_comb begin 
    if(transaction_queue.size() != 0) fifo_empty = 1'b0;
    else fifo_empty = 1'b1;
  end

  always_comb begin
    if (transaction_data.timestep == sim_time) begin
      pending_transaction = 1'b1;
    end else pending_transaction = 1'b0;

    if (transaction_data.timestep == sim_time && emitted_trans) begin
      transaction_queue.pop_front();
    end 
  end

  // Clock-driven logic
  always_comb begin
    current_req = '0;

    req_type = extract_transtype(transaction_data.line_data, "type");
    //$display("trans type: %s",req_type);
    // Decode request (AW or W supported at the moment)
    if (req_type == "AW") begin
      current_req.aw_valid = 1'b1;
      current_req.aw.addr  = extract_value(transaction_data.line_data, "addr");
      current_req.aw.id    = extract_value(transaction_data.line_data, "id");
      current_req.aw.burst = extract_value(transaction_data.line_data, "burst");
      current_req.aw.len   = extract_value(transaction_data.line_data, "len");
      current_req.aw.cache = extract_value(transaction_data.line_data, "cache");
      current_req.aw.lock  = extract_value(transaction_data.line_data, "lock");
      current_req.aw.prot  = extract_value(transaction_data.line_data, "prot");
      current_req.aw.qos   = extract_value(transaction_data.line_data, "qos");
      current_req.aw.size  = extract_value(transaction_data.line_data, "size");
      current_req.aw.user  = extract_value(transaction_data.line_data, "user");
    end else if (req_type == "W") begin
      current_req.w_valid = 1'b1;
      current_req.w.data  = extract_value(transaction_data.line_data, "data");
      current_req.w.strb  = extract_value(transaction_data.line_data, "strb");
      current_req.w.last  = extract_value(transaction_data.line_data, "last");
      current_req.w.user  = extract_value(transaction_data.line_data, "user");
      current_req.aw_valid = '0;
      current_req.aw.addr  = '0;
      current_req.aw.id    = '0;
      current_req.aw.burst = '0;
      current_req.aw.len   = '0;
      current_req.aw.cache = '0;
      current_req.aw.lock  = '0;
      current_req.aw.prot  = '0;
      current_req.aw.qos   = '0;
      current_req.aw.size  = '0;
      current_req.aw.user  = '0;
    end else if (req_type == "AR") begin
      current_req.ar_valid = 1'b1;
      current_req.ar.addr  = extract_value(transaction_data.line_data, "addr");
      current_req.ar.id    = extract_value(transaction_data.line_data, "id");
      current_req.ar.burst = extract_value(transaction_data.line_data, "burst");
      current_req.ar.len   = extract_value(transaction_data.line_data, "len");
      current_req.ar.cache = extract_value(transaction_data.line_data, "cache");
      current_req.ar.lock  = extract_value(transaction_data.line_data, "lock");
      current_req.ar.prot  = extract_value(transaction_data.line_data, "prot");
      current_req.ar.qos   = extract_value(transaction_data.line_data, "qos");
      current_req.ar.size  = extract_value(transaction_data.line_data, "size");
      current_req.ar.user  = extract_value(transaction_data.line_data, "user");
    end      //end
  end

  // State Transition Logic (Combinational)
  always_comb begin
    next_state = current_state;
    emitted_trans = 1'b0;
    debug =1'b0;
    axi_req_o = '0;
    case (current_state)
      IDLE: begin 
        if (sim_time) debug =1'b1;
        if(pending_transaction) begin
          axi_req_o = current_req;
          if (req_type == "AW") begin 
            if (axi_resp_i.aw_ready) begin 
              next_state = IDLE; 
              emitted_trans = 1'b1;
            end else next_state = ISSUE;  
          end else if (req_type == "W") begin 
            if (axi_resp_i.w_ready) begin 
              next_state = IDLE;
              emitted_trans = 1'b1; 
            end else next_state = ISSUE;  
          end else if (req_type == "R") begin 
            if (axi_resp_i.ar_ready) begin 
              next_state = IDLE; 
              emitted_trans = 1'b1;
            end else next_state = ISSUE;  
          end else next_state = IDLE;
        end else next_state = IDLE;
      end              
      ISSUE: begin
        axi_req_o = current_req;
        if (req_type == "AW") begin 
          if (axi_resp_i.aw_ready) begin 
            next_state = IDLE; 
            emitted_trans = 1'b1;
          end else next_state = ISSUE;  
        end else if (req_type == "W") begin 
          if (axi_resp_i.w_ready) begin 
            next_state = IDLE;
            emitted_trans = 1'b1; 
          end else next_state = ISSUE;  
        end else if (req_type == "R") begin 
          if (axi_resp_i.ar_ready) begin 
            next_state = IDLE; 
            emitted_trans = 1'b1;
          end else next_state = ISSUE;  
        end else axi_req_o = '0;  
      end 
      default: next_state = IDLE; // Default case for safety
    endcase
  end

  // State Register
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      current_state <= IDLE; 
    end else begin
      current_state <= next_state;
    end
  end

  // Helper function to extract the numeric value from a string field
  function logic [AxiDataWidth-1:0] extract_value(string source, string key);
    // Declare local variables
    string value_str;
    logic [AxiDataWidth-1:0] val;
    logic [AxiDataWidth-1:0] value_int;
    int i, j;
    automatic int key_len = key.len() - 1;
    automatic int source_len = source.len() - 1;

    //$display("look for %s in %s %d", key, source, source_len);
    // Start searching the source string for the key
    i = 0;
    while (i < source_len) begin
      // Check if we have found the key
      if (source.substr(i, key_len + i) == key) begin
        // Move index past the key and jump ': 
        if (key == "time") i = i + key_len + 19;
        else i = i + key_len + 4;

        //$display("The key '%s' cont %s i: %d", key, source.substr(i, i+10), i);
        // Now we start extracting the value
        j = i;
        // Find where the value ends (either comma, closing brace, or end of string)
        if (key != "time") begin
          while (j < source_len && source[j] != "," && source[j+1] != " " && source[j+2] != "'") j++;
        end else begin
          while (i < source_len && source[i] == " ") i++;
          while (j < source_len && source[j] != ",") j++;
        end 
        
        // Extract the value string from i to j-1
        value_str = source.substr(i, j);
        //$display("The key '%s' has a hex value: %s %d %d", key, value_str, i , j);
        // Check if the value is a hexadecimal number
        if (value_str[0] == "0" && value_str[1] == "x") begin
          // Convert hexadecimal to integer
          value_int = $sscanf(source.substr(i+2, j), "%h", val);
          //$display("The key '%s' has a hex value: %h", key, val);
        end else if (key == "time") begin // Check if the value is a decimal number
          // Convert decimal to integer
          value_int = $sscanf(value_str, "%d", val);
          //$display("The key '%s' has a hex value: %d", key, val);
        
        // Check if the value is a string (wrapped in single or double quotes)
        //else if (value_str[0] == "\"" && value_str[value_str.len()-1] == "\"") begin
        //  // For simplicity, return 0 for string values
        //  $display("The key '%s' has a string value: %s", key, value_str);
        //  value_int = 0;
        end else begin
          // If none of the above, return an error
          //$display("Unknown value type for key '%s': %s", key, value_str);
          val = -1;
        end

        // Return the value
        return val;
      end
      i = i + 1;
    end

    // If the key is not found, return an invalid value (e.g., -1)
    //$display("Key not found: %s", key);
    return -1;
  endfunction

  function string extract_transtype(string source, string key);
    // Declare local variables
    string value_str;
    string val;
    int i, j;
    automatic int key_len = key.len() - 1;
    automatic int source_len = source.len() - 1;

    // Start searching the source string for the key
    i = 0;
    while (i < source_len) begin
      // Check if we have found the key
      if (source.substr(i, key_len + i) == key) begin
        // Move index past the key and jump ': 
        if (key == "time") i = i + key_len + 19;
        else i = i + key_len + 4;
        
        // Now we start extracting the value
        j = i;
        // Find where the value ends (either comma, closing brace, or end of string)
        while (j < source_len && source[j] != "," && source[j+1] != " " && source[j+2] != "'") begin
          j = j + 1;
        end
        
        // Extract the value string from i to j-1
        value_str = source.substr(i, j);
        // Check if the value is a hexadecimal number
        if ( InjAW && value_str[1] == "A" && value_str[2] == "W" ) val = "AW";
        else if ( InjW && value_str[1] == "W" ) val = "W";
        else if ( InjAR && value_str[1] == "A" && value_str[2] == "R") val = "AR"; 
        else if ( InjR && value_str[1] == "R" ) val = "R";
        else val = "NONE";

        // Return the value
        return val;
      end
      i = i + 1;
    end

    // If the key is not found, return an invalid value (e.g., -1)
    //$display("Key not found: %s", key);
    return "NONE";
  endfunction

  function int find(string source, string search);
    int i;
    for (i = 0; i <= source.len() - search.len(); i++) begin
      if (source.substr(i, search.len()) == search) begin
        return i;
      end
    end
    return -1; // Not found
  endfunction

endmodule