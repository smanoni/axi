// Copyright 2020 ETH Zurich and University of Bologna.
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
// - Matheus Cavalcante <matheusd@iis.ee.ethz.ch>
// - Andreas Kurth <akurth@iis.ee.ethz.ch>

`include "axi/assign.svh"

module tb_axi_dw_downsizer #(
    // AXI Parameters
    parameter int unsigned TbAddrWidth        = 64  ,
    parameter int unsigned TbIdWidth          = 4   ,
    parameter int unsigned TbSlvPortDataWidth = 64  ,
    parameter int unsigned TbMstPortDataWidth = 32  ,
    parameter int unsigned TbUserWidth        = 8   ,
    // TB Parameters
    parameter time TbCyclTime                 = 10ns,
    parameter time TbApplTime                 = 2ns ,
    parameter time TbTestTime                 = 8ns
  );

  /*********************
   *  CLOCK GENERATOR  *
   *********************/

  logic clk;
  logic rst_n;
  logic eos;

  clk_rst_gen #(
    .ClkPeriod    (TbCyclTime),
    .RstClkCycles (5       )
  ) i_clk_rst_gen (
    .clk_o (clk  ),
    .rst_no(rst_n)
  );

  /*********
   *  AXI  *
   *********/

  // Master port

  AXI_BUS_DV #(
    .AXI_ADDR_WIDTH(TbAddrWidth       ),
    .AXI_DATA_WIDTH(TbSlvPortDataWidth),
    .AXI_ID_WIDTH  (TbIdWidth         ),
    .AXI_USER_WIDTH(TbUserWidth       )
  ) master_dv (
    .clk_i(clk)
  );

  AXI_BUS #(
    .AXI_ADDR_WIDTH(TbAddrWidth       ),
    .AXI_DATA_WIDTH(TbSlvPortDataWidth),
    .AXI_ID_WIDTH  (TbIdWidth         ),
    .AXI_USER_WIDTH(TbUserWidth       )
  ) master ();

  `AXI_ASSIGN(master, master_dv)

  axi_test::axi_rand_master #(
    .AW             (TbAddrWidth       ),
    .DW             (TbSlvPortDataWidth),
    .IW             (TbIdWidth         ),
    .UW             (TbUserWidth       ),
    .TA             (TbApplTime        ),
    .TT             (TbTestTime        ),
    .MAX_READ_TXNS  (8                 ),
    .MAX_WRITE_TXNS (8                 ),
    .AXI_BURST_FIXED(1'b0              ),
    .AXI_ATOPS      (1'b1              )
  ) master_drv = new (master_dv);

  // Slave port

  AXI_BUS_DV #(
    .AXI_ADDR_WIDTH(TbAddrWidth       ),
    .AXI_DATA_WIDTH(TbMstPortDataWidth),
    .AXI_ID_WIDTH  (TbIdWidth         ),
    .AXI_USER_WIDTH(TbUserWidth       )
  ) slave_dv (
    .clk_i(clk)
  );

  AXI_BUS #(
    .AXI_ADDR_WIDTH(TbAddrWidth       ),
    .AXI_DATA_WIDTH(TbMstPortDataWidth),
    .AXI_ID_WIDTH  (TbIdWidth         ),
    .AXI_USER_WIDTH(TbUserWidth       )
  ) slave ();

  axi_test::axi_rand_slave #(
    .AW(TbAddrWidth       ),
    .DW(TbMstPortDataWidth),
    .IW(TbIdWidth         ),
    .UW(TbUserWidth       ),
    .TA(TbApplTime        ),
    .TT(TbTestTime        )
  ) slave_drv = new (slave_dv);

  `AXI_ASSIGN(slave_dv, slave)

  /*********
   *  DUT  *
   *********/

  axi_dw_converter_intf #(
    .AXI_MAX_READS          (4                 ),
    .AXI_ADDR_WIDTH         (TbAddrWidth       ),
    .AXI_ID_WIDTH           (TbIdWidth         ),
    .AXI_SLV_PORT_DATA_WIDTH(TbSlvPortDataWidth),
    .AXI_MST_PORT_DATA_WIDTH(TbMstPortDataWidth),
    .AXI_USER_WIDTH         (TbUserWidth       )
  ) i_dw_converter (
    .clk_i (clk   ),
    .rst_ni(rst_n ),
    .slv   (master),
    .mst   (slave )
  );

  /*************
   *  DRIVERS  *
   *************/

  initial begin
    eos = 1'b0;

    // Configuration
    slave_drv.reset()                                                                                ;
    master_drv.reset()                                                                               ;
    master_drv.add_memory_region({TbAddrWidth{1'b0}}, {TbAddrWidth{1'b1}}, axi_pkg::WTHRU_NOALLOCATE);

    // Wait for the reset before sending requests
    @(posedge rst_n);

    fork
      // Act as a sink
      slave_drv.run()         ;
      master_drv.run(200, 200);
    join_any

    // Done
    repeat (10) @(posedge clk);
    eos = 1'b1;
  end

  /*************
   *  MONITOR  *
   *************/

  initial begin : proc_monitor
    static tb_axi_dw_pkg::axi_dw_downsizer_monitor #(
      .AddrWidth       (TbAddrWidth       ),
      .MstPortDataWidth(TbMstPortDataWidth),
      .SlvPortDataWidth(TbSlvPortDataWidth),
      .IdWidth         (TbIdWidth         ),
      .UserWidth       (TbUserWidth       ),
      .TimeTest        (TbTestTime        )
    ) monitor = new (master_dv, slave_dv);
    fork
      monitor.run();
      forever begin
        #TbTestTime;
        if(eos) begin
          monitor.print_result();
          $stop()               ;
        end
        @(posedge clk);
      end
    join
  end

// vsim -voptargs=+acc work.tb_axi_dw_downsizer
endmodule : tb_axi_dw_downsizer
