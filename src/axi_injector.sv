// Copyright (c) 2024 ETH Zurich, University of Bologna
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
  parameter      SizeQueue  = 3'd4,
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

  transaction_t aw_trans_queue[$:SizeQueue]; // Queue of transactions
  transaction_t ar_trans_queue[$:SizeQueue];
  transaction_t w_trans_queue[$:SizeQueue];

  axi_req_t      current_req;  // Holds the current request being emitted
  transaction_t  aw_trans_data, ar_trans_data, w_trans_data;
  transaction_t  aw_trans, ar_trans, w_trans;
  transaction_t  w_trans0, w_trans1, w_trans2, w_trans3; 

  logic aw_pending, ar_pending, w_pending;
  logic aw_req, w_req, ar_req;
  int sim_time;
  bit file_line_ack;
  string req_type;
  logic aw_pop, w_pop, ar_pop;

  always_comb begin
    w_trans_data.timestep = w_trans_queue[0].timestep;
    w_trans_data.line_data = w_trans_queue[0].line_data;
    aw_trans_data.timestep = aw_trans_queue[0].timestep;
    aw_trans_data.line_data = aw_trans_queue[0].line_data;
    ar_trans_data.timestep = ar_trans_queue[0].timestep;
    ar_trans_data.line_data = ar_trans_queue[0].line_data;

    w_trans0.timestep  = w_trans_queue[0].timestep;
    w_trans0.line_data = w_trans_queue[0].line_data;
    w_trans1.timestep  = w_trans_queue[1].timestep;
    w_trans1.line_data = w_trans_queue[1].line_data;
    w_trans2.timestep  = w_trans_queue[2].timestep;
    w_trans2.line_data = w_trans_queue[2].line_data;
    w_trans3.timestep  = w_trans_queue[3].timestep;
    w_trans3.line_data = w_trans_queue[3].line_data;
  end

  initial begin
    string file_line;
    int file;
    // If is true the prev line has been consumed
    file_line_ack = 1'b1;

    // Open file
    file = $fopen(FileName, "r");
    if (file == 0) begin
      $fatal(1, "Failed to open file: %s", FileName);
    end

    // Start reading lines from file
    while (!$feof(file)) begin
  
      //$display("aw: %d, ar: %d, w: %d ",aw_trans_queue.size(), ar_trans_queue.size(), w_trans_queue.size());
      if (file_line_ack == 1'b1) begin
        file_line = "";
        $fgets(file_line, file);  
      end
      //$display("fl: %s time %d", file_line,$time);
      //$display("aw %d ar %d w %d %d time %d", aw_trans_queue.size(), ar_trans_queue.size(), w_trans_queue.size(), SizeQueue,$time);
      // Check type of AXI transaction
      req_type = extract_transtype(file_line, "type");
      case(req_type)
        "AW": begin
          if (aw_trans_queue.size() < SizeQueue) begin
            aw_trans.timestep  = extract_value(file_line, "time");
            aw_trans.line_data = file_line;
            aw_trans_queue.push_back(aw_trans);
            //aw_trans_data.timestep = aw_trans_queue[0].timestep;
            //aw_trans_data.line_data = aw_trans_queue[0].line_data;
            file_line_ack = 1'b1; 
          end else begin 
            file_line_ack = 1'b0;           
          end
        end
        "AR": begin
          if (ar_trans_queue.size() < SizeQueue) begin
            ar_trans.timestep  = extract_value(file_line, "time");
            ar_trans.line_data = file_line;
            ar_trans_queue.push_back(ar_trans);
            //ar_trans_data.timestep = ar_trans_queue[0].timestep;
            //ar_trans_data.line_data = ar_trans_queue[0].line_data;
            file_line_ack = 1'b1; 
          end else begin 
            file_line_ack = 1'b0;          
          end
        end
        "W": begin
          if (w_trans_queue.size() < SizeQueue) begin
            w_trans.timestep  = extract_value(file_line, "time");
            w_trans.line_data = file_line;
            w_trans_queue.push_back(w_trans);
            //$display("t: %d, fl: %s", $time, w_trans_queue[3].line_data);
            //w_trans_data.timestep = w_trans_queue[0].timestep;
            //w_trans_data.line_data = w_trans_queue[0].line_data;
            file_line_ack = 1'b1; 
          end else begin 
            file_line_ack = 1'b0;
          end
        end
        "NONE": begin
          file_line_ack = 1'b1;  
        end
      endcase  
      //$display("tran queue %d",transaction_data.timestep);
      @(posedge clk_i);
    end  
  end

  // Simulation clocks
  always_ff @(posedge clk_i) begin
    sim_time <= $time + 10;
  end 
//&& aw_trans_data.timestep != '0
 //&& w_trans_data.timestep != '0
 //&& ar_trans_data.timestep != '0
  always_comb begin
    // AW AXI pending request
    if (!rst_ni) begin
      aw_req = 1'b0;
      ar_req = 1'b0;
      w_req  = 1'b0;
    end else begin
      if (aw_trans_data.timestep <= sim_time && aw_trans_queue.size() != 0) begin
        aw_req = 1'b1;
      end else aw_req = 1'b0;
  
      // Pending AR AXI request
      if (ar_trans_data.timestep <= sim_time && ar_trans_queue.size() != 0) begin
        ar_req = 1'b1;
      end else ar_req = 1'b0;
  
      if (w_trans_data.timestep <= sim_time && w_trans_queue.size() != 0) begin
        w_req = 1'b1;
        //$display("here w_req %d", $time);
        //$display("here w_req %d time %d", w_trans_data.timestep, sim_time);
      end else w_req = 1'b0;
    end 
  end 

  assign aw_pop = (aw_req || aw_pending) && axi_resp_i.aw_ready;
  assign w_pop = (w_req || w_pending) && axi_resp_i.w_ready;
  assign ar_pop = (ar_req || ar_pending) && axi_resp_i.ar_ready;

  always @(posedge clk_i) begin
    if ((aw_req || aw_pending) && axi_resp_i.aw_ready) begin
      //$display("here aw_req %d", $time);
      //$display("queue size %d", aw_trans_queue.size());
      aw_trans_queue.pop_front();
      //$display("queue size %d", aw_trans_queue.size());
    end 
    
    if ((w_req || w_pending) && axi_resp_i.w_ready) begin
      //$display("queue size %d", w_trans_queue.size());
      //$display("here w_req %d", $time);
      w_trans_queue.pop_front();
      //$display("queue size %d", w_trans_queue.size());
    end 
    
    if ((ar_req || ar_pending) && axi_resp_i.ar_ready) begin
      ar_trans_queue.pop_front();
    end 
  end

  //always @(pos) begin
  //    if (axi_req_o.aw_valid && axi_resp_i.aw_ready) begin
  //      //$display("here aw_req %d", $time);
  //      aw_trans_queue.pop_front();
  //    end 
  //    
  //    if (axi_req_o.w_valid && axi_resp_i.w_ready) begin
  //      //$display("here w_req %d", $time);
  //      w_trans_queue.pop_front();
  //    end 
  //    
  //    if (axi_req_o.ar_valid && axi_resp_i.ar_ready) begin
  //      ar_trans_queue.pop_front();
  //    end 
  //  //end
  //end

  always_ff @(posedge clk_i or negedge rst_ni) begin : proc_pending_req
    if(~rst_ni) begin
      aw_pending <= 1'b0;
      w_pending  <= 1'b0;
      ar_pending <= 1'b0;
    end else begin
      if (aw_req && !axi_resp_i.aw_ready) begin
        aw_pending <= 1'b1; 
      end else if (axi_resp_i.aw_ready) aw_pending <= 1'b0;

      if (w_req && !axi_resp_i.w_ready) begin
        w_pending <= 1'b1; 
      end else if (axi_resp_i.w_ready) begin 
        w_pending <= 1'b0;
      end 

      if (ar_req && !axi_resp_i.ar_ready) begin
        ar_pending <= 1'b1; 
      end else if (axi_resp_i.ar_ready) ar_pending <= 1'b0;

      //if ((aw_req || aw_pending) && axi_resp_i.aw_ready) begin
      //  //$display("here aw_req %d", $time);
      //  aw_trans_queue.pop_front();
      //end 
      //if ((w_req || w_pending) && axi_resp_i.w_ready) begin
      //  //$display("here w_req %d", $time);
      //  w_trans_queue.pop_front();
      //end 
      //if ((ar_req || ar_pending) && axi_resp_i.ar_ready) begin
      //  ar_trans_queue.pop_front();
      //end 
    end
  end

  // Clock-driven logic
  always_comb begin
    axi_req_o = '0;
    if (rst_ni) begin
      axi_req_o.b_ready = 1'b1;
      axi_req_o.r_ready = 1'b1;
    end
    //$display("trans type: %s",req_type);
    // Decode request (AW or W supported at the moment)
    if (aw_req || aw_pending) begin
      axi_req_o.aw_valid = 1'b1;
      axi_req_o.aw.addr  = extract_value(aw_trans_data.line_data, "addr");
      axi_req_o.aw.id    = extract_value(aw_trans_data.line_data, "id");
      axi_req_o.aw.burst = extract_value(aw_trans_data.line_data, "burst");
      axi_req_o.aw.len   = extract_value(aw_trans_data.line_data, "len");
      axi_req_o.aw.cache = extract_value(aw_trans_data.line_data, "cache");
      axi_req_o.aw.lock  = extract_value(aw_trans_data.line_data, "lock");
      axi_req_o.aw.prot  = extract_value(aw_trans_data.line_data, "prot");
      axi_req_o.aw.qos   = extract_value(aw_trans_data.line_data, "qos");
      axi_req_o.aw.size  = extract_value(aw_trans_data.line_data, "size");
      axi_req_o.aw.user  = extract_value(aw_trans_data.line_data, "user");
    end 

    if (w_req || w_pending) begin
      axi_req_o.w_valid = 1'b1;
      axi_req_o.w.data  = extract_value(w_trans_data.line_data, "data");
      axi_req_o.w.strb  = extract_value(w_trans_data.line_data, "strb");
      axi_req_o.w.last  = extract_value(w_trans_data.line_data, "last");
      axi_req_o.w.user  = extract_value(w_trans_data.line_data, "user");
    end 

    if (ar_req || ar_pending) begin
      axi_req_o.ar_valid = 1'b1;
      axi_req_o.ar.addr  = extract_value(ar_trans_data.line_data, "addr");
      axi_req_o.ar.id    = extract_value(ar_trans_data.line_data, "id");
      axi_req_o.ar.burst = extract_value(ar_trans_data.line_data, "burst");
      axi_req_o.ar.len   = extract_value(ar_trans_data.line_data, "len");
      axi_req_o.ar.cache = extract_value(ar_trans_data.line_data, "cache");
      axi_req_o.ar.lock  = extract_value(ar_trans_data.line_data, "lock");
      axi_req_o.ar.prot  = extract_value(ar_trans_data.line_data, "prot");
      axi_req_o.ar.qos   = extract_value(ar_trans_data.line_data, "qos");
      axi_req_o.ar.size  = extract_value(ar_trans_data.line_data, "size");
      axi_req_o.ar.user  = extract_value(ar_trans_data.line_data, "user");
    end      //end
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
        //if (key == "time") i = i + key_len + 19;
        //else i = i + key_len + 4;
        i = i + key_len + 4;

        //$display("The key '%s' cont %s i: %d", key, source.substr(i, i+10), i);
        // Now we start extracting the value
        j = i;
        // Find where the value ends (either comma, closing brace, or end of string)
        if (key != "time") begin
          //find first number of time
          while (i < source_len && source[i] == " ") i++;
          j = i;
          while (j < source_len && source[j] != "," && source[j+1] != " " && source[j+2] != "'") j++;
        end else begin
          while (i < source_len && source[i] == " ") i++;
          while (j < source_len && source[j] != ",") j++;
        end 
        
        // Extract the value string from i to j-1
        value_str = source.substr(i, j);
        // Check if the value is a hexadecimal number
        if (value_str[0] == "0" && value_str[1] == "x") begin
          // Convert hexadecimal to integer
          value_int = $sscanf(source.substr(i+2, j), "%h", val);
          //$display("The key '%s' has a hex value: %h", key, val);
        end else if (key == "time") begin // Check if the value is a decimal number
          // Convert decimal to integer
          value_int = $sscanf(value_str, "%d", val);
          //$display("The key '%s' has a hex value: %s %d %d", key, value_str, i , j);
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