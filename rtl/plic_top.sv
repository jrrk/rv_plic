module plic_top #(
  parameter int N_SOURCE    = 30,
  parameter int N_TARGET    = 2,
  parameter int MAX_PRIO    = 7,
  parameter int SRCW        = $clog2(N_SOURCE+1)
) (
  input  logic clk_i,    // Clock
  input  logic rst_ni,  // Asynchronous reset active low
  input wire         valid_fence_i_r_i,   
`ifdef XLNX_ILA_PLIC   
  output wire trig_out, // ILA debugging triggers
  input wire trig_out_ack,
`endif
  // Bus Interface
  input  reg_intf::reg_intf_req_a32_d32 req_i,
  output reg_intf::reg_intf_resp_d32    resp_o,
  input logic [N_SOURCE-1:0] le_i, // 0:level 1:edge
  // Interrupt Sources
  input  logic [N_SOURCE-1:0] irq_sources_i,
  // Interrupt notification to targets
  output logic [N_TARGET-1:0] eip_targets_o
);
  localparam PRIOW = $clog2(MAX_PRIO+1);

  logic [N_SOURCE-1:0] ip;
  logic [N_SOURCE-1:0] ia;

  logic [N_TARGET-1:0][PRIOW-1:0]    threshold_q;

  logic [N_TARGET-1:0]               claim_re, claim_re_d, claim_re_q; //Target read indicator
  logic [N_TARGET-1:0][SRCW-1:0]     claim_id;
  logic [N_SOURCE-1:0]               claim; //Converted from claim_re/claim_id
  logic [N_SOURCE-1:0][25:0]         claim_timeout_d, claim_timeout_q;
   
  logic [N_TARGET-1:0]               complete_we; //Target write indicator
  logic [N_TARGET-1:0][SRCW-1:0]     complete_id;
  logic [N_SOURCE-1:0]               complete; //Converted from complete_re/complete_id

  logic [N_SOURCE-1:0][PRIOW-1:0]    prio_q;
  logic [N_TARGET-1:0][N_SOURCE-1:0] ie_q;

  logic [N_TARGET-1:0][4:0]          defer_cnt_d, defer_cnt_q;
  logic [N_TARGET-1:0]               defer_complete;

  always_comb begin
     claim = '0;
     complete = '0;
     for (int i = 0 ; i < N_SOURCE ; i++) begin
        if (claim_timeout_q[i])
          begin
             if (claim_timeout_q[i] == 1)
               begin
                  complete[i] = 1'b1;
                  claim_timeout_d[i] = 0;
               end
             else
               claim_timeout_d[i] = claim_timeout_q[i] - 1;
          end
        else 
          claim_timeout_d[i] = 0;
        end
     for (int i = 0 ; i < N_TARGET ; i++) begin
        defer_complete[i] = &(defer_cnt_q[i]);
        defer_cnt_d[i] = defer_cnt_q[i] + !(defer_complete[i]);
        if (defer_complete[i]) // if we don't see the fence, flush the claim after a while ...
          claim_re_d[i] = '0;
        else
          claim_re_d[i] = claim_re_q[i];
        if (claim_re[i] && claim_id[i] != 0)
          begin
             claim_re_d[i] = 1'b1;
             defer_cnt_d[i] = '0; // defer further interrupts for a while
          end
        if (claim_re_q[i] && valid_fence_i_r_i) // CPU accepted the claim, process it
          begin
             claim[claim_id[i]-1] = 1'b1;
             claim_timeout_d[claim_id[i]-1] = 50000000;
             claim_re_d[i] = '0;
          end
        if (complete_we[i] && complete_id[i] != 0)
          begin
             complete[complete_id[i]-1] = 1'b1;
             claim_timeout_d[complete_id[i]-1] = 0;
          end
     end
  end

  // Gateways
  rv_plic_gateway #(
    .N_SOURCE (N_SOURCE)
  ) i_rv_plic_gateway (
    .clk_i,
    .rst_ni,
    .src(irq_sources_i),
    .le(le_i),
    .claim(claim),
    .complete(complete),
    .ip(ip),
    .ia(ia)
  );

  // Target interrupt notification
  for (genvar i = 0 ; i < N_TARGET; i++) begin : gen_target
    rv_plic_target #(
      .N_SOURCE  ( N_SOURCE ),
      .MAX_PRIO  ( MAX_PRIO ),
      .ALGORITHM ( "SEQUENTIAL" )
    ) i_target (
      .clk_i,
      .rst_ni,
      .ip(ip),
      .ie(ie_q[i]),
      .prio(prio_q),
      .threshold(threshold_q[i]),
      .irq(eip_targets_o[i]),
      .irq_id(claim_id[i])
    );
  end

  logic [N_TARGET-1:0] threshold_we_o;
  logic [N_TARGET-1:0][PRIOW-1:0] threshold_o;

  logic [N_SOURCE:0][PRIOW-1:0] prio_i, prio_o;
  logic [N_SOURCE:0] prio_we_o;

  // TODO(zarubaf): This needs more graceful handling
  // it will break if the number of sources is larger than 32
  logic [N_TARGET-1:0][N_SOURCE:0] ie_i, ie_o;
  logic [N_TARGET-1:0] ie_we_o;

  plic_regs i_plic_regs (
    .prio_i(prio_i),
    .prio_o(prio_o),
    .prio_we_o(prio_we_o),
    .prio_re_o(), // don't care
    // source zero is always zero
    .ip_i({ip, 1'b0}),
    .ip_re_o(), // don't care
    .ie_i(ie_i),
    .ie_o(ie_o),
    .ie_we_o(ie_we_o),
    .ie_re_o(), // don't care
    .threshold_i(threshold_q),
    .threshold_o(threshold_o),
    .threshold_we_o(threshold_we_o),
    .threshold_re_o(), // don't care
    .cc_i(claim_id),
    .cc_o(complete_id),
    .cc_we_o(complete_we),
    .cc_re_o(claim_re),
    .req_i,
    .resp_o
  );

  assign prio_i[0] = '0;

  for (genvar i = 0; i < N_TARGET; i++) begin
    assign ie_i[i] = {ie_q[i][N_SOURCE-1:0], 1'b0};
  end

  for (genvar i = 1; i < N_SOURCE + 1; i++) begin
    assign prio_i[i] = prio_q[i - 1];
  end

  // registers
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (~rst_ni) begin
      prio_q <= '0;
      ie_q <= '0;
      threshold_q <= '0;
      defer_cnt_q <= '0;
      claim_re_q <= '0;
      claim_timeout_q <= '0;
    end else begin
      defer_cnt_q <= defer_cnt_d;
      claim_re_q <= claim_re_d;
      // source zero is 0
      for (int i = 0; i < N_SOURCE; i++) begin
        claim_timeout_q[i] <= claim_timeout_d[i];
        prio_q[i] <= prio_we_o[i + 1] ? prio_o[i + 1] : prio_q[i];
      end
      for (int i = 0; i < N_TARGET; i++) begin
        threshold_q[i] <= threshold_we_o[i] ? threshold_o[i] : threshold_q[i];
        ie_q[i] <= ie_we_o[i] ? ie_o[i][N_SOURCE:1] : ie_q[i];
      end

    end
  end

`ifdef XLNX_ILA_PLIC   
xlnx_ila_plic plic_ila (
	.clk(clk_i),
        .trig_out(trig_out),// output wire trig_out 
        .trig_out_ack(trig_out_ack),// input wire trig_out_ack 
	.probe0(prio_i[3:0]),
	.probe1(prio_o[3:0]),
	.probe2(prio_we_o[3:0]),
	.probe3(ip[3:0]),
	.probe4(ie_i),
	.probe5(ie_o),
	.probe6(ie_we_o),
	.probe7(threshold_q),
	.probe8(threshold_o),
	.probe9(threshold_we_o),
	.probe10(claim_id),
	.probe11(complete_id),
	.probe12(complete_we),
	.probe13(claim_re),
	.probe14(req_i),
	.probe15(resp_o),
	.probe16(claim[3:0]),
	.probe17(complete[3:0]),
	.probe18(prio_q[3:0]),
	.probe19(ie_q),
        .probe20(irq_sources_i[3:0]),
        .probe21(eip_targets_o),
        .probe22(defer_cnt_d),
        .probe23(defer_cnt_q),
        .probe24(defer_complete),
        .probe25(valid_fence_i_r_i),
        .probe26(claim_re_d),
        .probe27(claim_re_q),
	.probe28(ia[3:0]),
        .probe29(claim_timeout_d[3:0]),
        .probe30(claim_timeout_q[3:0])
);
`endif
   
endmodule
