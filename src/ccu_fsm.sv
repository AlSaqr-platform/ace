`include "ace/assign.svh"
`include "ace/typedef.svh"

module ccu_fsm
#(
    parameter int unsigned DcacheLineWidth = 0,
    parameter int unsigned AxiDataWidth = 0,
    parameter int unsigned NoMstPorts = 4,
    parameter int unsigned SlvAxiIDWidth = 0,
    parameter type mst_req_t    = logic,
    parameter type mst_resp_t   = logic,
    parameter type snoop_req_t  = logic,
    parameter type snoop_resp_t = logic
) (
    //clock and reset
    input                               clk_i,
    input                               rst_ni,
    // CCU Request In and response out
    input  mst_req_t                    ccu_req_i,
    output mst_resp_t                   ccu_resp_o,
    //CCU Request Out and response in
    output mst_req_t                    ccu_req_o,
    input  mst_resp_t                   ccu_resp_i,
    // Snoop channel resuest and response
    output snoop_req_t  [NoMstPorts-1:0] s2m_req_o,
    input  snoop_resp_t [NoMstPorts-1:0] m2s_resp_i
);

    localparam int unsigned DcacheLineWords = DcacheLineWidth / AxiDataWidth;
    localparam int unsigned MstIdxBits      = $clog2(NoMstPorts);

    enum logic [3:0] {
      IDLE,                      // 0
      SEND_INVALID_R,            // 1
      WAIT_INVALID_R,            // 2
      SEND_AXI_REQ_WRITE_BACK_R, // 3
      WRITE_BACK_MEM_R,          // 4
      SEND_READ,                 // 5
      WAIT_RESP_R,               // 6
      READ_SNP_DATA,             // 7
      SEND_AXI_REQ_R,            // 8
      READ_MEM,                  // 9
      SEND_INVALID_W,            // 10
      WAIT_INVALID_W,            // 11
      SEND_AXI_REQ_WRITE_BACK_W, // 12
      WRITE_BACK_MEM_W,          // 13
      SEND_AXI_REQ_W,            // 14
      WRITE_MEM                  // 15
    } state_d, state_q;


    // snoop resoponse valid
    logic [NoMstPorts-1:0]          cr_handshake_holder_d, cr_handshake_holder_q;
    // snoop channel ac ready
    logic [NoMstPorts-1:0]          ac_handshake_holder_d, ac_handshake_holder_q;
    // snoop channel cd last
    logic [NoMstPorts-1:0]          cd_last_d, cd_last_q;
    // check for availablilty of data
    logic [NoMstPorts-1:0]          data_available_d, data_available_q;
    // check for response error
    logic [NoMstPorts-1:0]          response_error_d, response_error_q;
    // check for data received
    logic [NoMstPorts-1:0]          data_received_q, data_received_d;
    // check for shared in cr_resp
    logic [NoMstPorts-1:0]          shared_d, shared_q;
    // check for dirty in cr_resp
    logic [NoMstPorts-1:0]          dirty_d, dirty_q;
    // request holder
    mst_req_t                       ccu_req_holder_q;
    // snoop response holder
    snoop_resp_t [NoMstPorts-1:0]   m2s_resp_holder_q;
    // initiating master port
    logic [NoMstPorts-1:0]          initiator_d, initiator_q;
    logic [MstIdxBits-1:0]          first_responder_q;

    logic [DcacheLineWords-1:0][AxiDataWidth-1:0] cd_data_d, cd_data_q;
    logic [$clog2(DcacheLineWords+1)-1:0]         stored_cd_data_q;

    logic r_last;
    logic w_last;
    logic r_eot;
    logic w_eot;

    typedef struct packed {
      logic waiting_w;
      logic waiting_r;
    } prio_t;

    prio_t prio_d, prio_q;

    // -----------------------
    // Quality of Life Signals
    // -----------------------

    // These signals are added to give meaningful names to specific events

    logic slv_ar_handshake, slv_aw_handshake, slv_r_handshake;
    assign slv_ar_handshake = ccu_req_i.ar_valid & ccu_resp_o.ar_ready;
    assign slv_aw_handshake = ccu_req_i.aw_valid & ccu_resp_o.aw_ready;
    assign slv_r_handshake  = ccu_resp_o.r_valid & ccu_req_i.r_ready;

    logic mst_w_handshake;
    assign mst_w_handshake = ccu_req_o.w_valid & ccu_resp_i.w_ready;

    logic send_invalid_r;
    assign send_invalid_r = ccu_req_i.ar.snoop == snoop_pkg::CLEAN_UNIQUE || ccu_req_i.ar.lock;

    logic [NoMstPorts-1:0] ac_handshake;
    for (genvar i = 0; i < NoMstPorts; i = i + 1) begin
      assign ac_handshake[i] = (m2s_resp_i[i].ac_ready & s2m_req_o[i].ac_valid);
    end

    logic [NoMstPorts-1:0] cr_handshake;
    for (genvar i = 0; i < NoMstPorts; i = i + 1) begin
      assign cr_handshake[i] = (m2s_resp_i[i].cr_valid & s2m_req_o[i].cr_ready);
    end

    snoop_req_t  responder_req;
    snoop_resp_t responder_resp;
    logic responder_cd_handshake;
    logic cd_valid_data;

    assign responder_req  = s2m_req_o[first_responder_q];
    assign responder_resp = m2s_resp_i[first_responder_q];
    assign responder_cd_handshake = responder_resp.cd_valid & responder_req.cd_ready;
    assign cd_valid_data = (stored_cd_data_q != 0) || responder_cd_handshake;

    // ----------------------
    // Current State Block
    // ----------------------
    always_ff @(posedge clk_i, negedge rst_ni) begin : ccu_present_state
        if(!rst_ni) begin
            state_q <= IDLE;
            initiator_q <= '0;
            prio_q <= '0;
        end else begin
            state_q <= state_d;
            initiator_q <= initiator_d;
            prio_q <= prio_d;
        end
    end

    // ----------------------
    // Next State Block
    // ----------------------
    always_comb begin : ccu_state_ctrl

        state_d = state_q;
        initiator_d = initiator_q;
        prio_d = prio_q;

        case(state_q)

        IDLE: begin
            initiator_d = '0;
            prio_d = '0;
            //  wait for incoming valid request from master
            if(slv_ar_handshake) begin
              state_d = send_invalid_r ? SEND_INVALID_R : SEND_READ;
              initiator_d[ccu_req_i.ar.id[SlvAxiIDWidth+:MstIdxBits]] = 1'b1;
              prio_d.waiting_w = ccu_req_i.aw_valid;
            end else if(slv_aw_handshake) begin
              state_d = SEND_INVALID_W;
              initiator_d[ccu_req_i.aw.id[SlvAxiIDWidth+:MstIdxBits]] = 1'b1;
              prio_d.waiting_r = ccu_req_i.ar_valid;
            end
        end

        //---------------------
        //---- Read Branch ----
        //---------------------

        SEND_INVALID_R: begin
            // wait for all snoop masters to assert AC ready
            if (ac_handshake_holder_d != '1) begin
                state_d = SEND_INVALID_R;
            end else begin
                state_d = WAIT_INVALID_R;
            end
        end

        WAIT_INVALID_R: begin
            // wait for all snoop masters to assert CR valid
            if ((cr_handshake_holder_d == '1) && (ccu_req_i.r_ready || ccu_req_holder_q.ar.lock)) begin
                if(|(data_available_d & ~response_error_d)) begin
                    state_d = SEND_AXI_REQ_WRITE_BACK_R;
                end else begin
                    if (ccu_req_holder_q.ar.lock) begin   // AMO LR, read memory
                        state_d = SEND_AXI_REQ_R;
                    end else begin
                        state_d = IDLE;
                    end
                end
            end else begin
                state_d = WAIT_INVALID_R;
            end
        end

        SEND_AXI_REQ_WRITE_BACK_R: begin
            // wait for responding slave to assert aw_ready
            if(ccu_resp_i.aw_ready !='b1) begin
                state_d = SEND_AXI_REQ_WRITE_BACK_R;
            end else begin
                state_d = WRITE_BACK_MEM_R;
            end
        end

        WRITE_BACK_MEM_R: begin
            // wait for responding slave to send b_valid
            if((ccu_resp_i.b_valid && ccu_req_o.b_ready)) begin
                if (ccu_req_holder_q.ar.lock) begin   // AMO LR, read memory
                    state_d = SEND_AXI_REQ_R;
                end else begin
                    state_d = IDLE;
                end
            end else begin
                state_d = WRITE_BACK_MEM_R;
            end
        end

        SEND_READ: begin
            // wait for all snoop masters to de-assert AC ready
            if (ac_handshake_holder_d != '1) begin
                state_d = SEND_READ;
            end else begin
                state_d = WAIT_RESP_R;
            end
        end

        WAIT_RESP_R: begin
            // wait for all snoop masters to assert CR valid
            if (cr_handshake_holder_d != '1) begin
                state_d = WAIT_RESP_R;
            end else if(|(data_available_d & ~response_error_d)) begin
                state_d = READ_SNP_DATA;
            end else begin
                state_d = SEND_AXI_REQ_R;
            end
        end

        READ_SNP_DATA: begin
          if(cd_last_d == data_available_q && (r_eot == 1'b1 || (ccu_req_i.r_ready == 1'b1 && r_last == 1'b1))) begin
            state_d = IDLE;
          end else begin
            state_d = READ_SNP_DATA;
          end
        end

        SEND_AXI_REQ_R: begin
            // wait for responding slave to assert ar_ready
            if(ccu_resp_i.ar_ready !='b1) begin
                state_d = SEND_AXI_REQ_R;
            end else begin
                state_d = READ_MEM;
            end
        end

        READ_MEM: begin
            // wait for responding slave to assert r_valid
            if(ccu_resp_i.r_valid && ccu_req_i.r_ready) begin
                if(ccu_resp_i.r.last) begin
                    state_d = IDLE;
                end else begin
                    state_d = READ_MEM;
                end
            end else begin
                state_d = READ_MEM;
            end
        end


        //---------------------
        //---- Write Branch ---
        //---------------------

        SEND_INVALID_W: begin
            // wait for all snoop masters to assert AC ready
            if (ac_handshake_holder_d != '1) begin
                state_d = SEND_INVALID_W;
            end else begin
                state_d = WAIT_INVALID_W;
            end
        end

        WAIT_INVALID_W: begin
            // wait for all snoop masters to assert CR valid
            if (cr_handshake_holder_d != '1) begin
                state_d = WAIT_INVALID_W;
            end else if(|(data_available_d & ~response_error_d)) begin
                state_d = SEND_AXI_REQ_WRITE_BACK_W;
            end else begin
                state_d = SEND_AXI_REQ_W;
            end
        end

        SEND_AXI_REQ_WRITE_BACK_W: begin
            // wait for responding slave to assert aw_ready
            if(ccu_resp_i.aw_ready !='b1) begin
                state_d = SEND_AXI_REQ_WRITE_BACK_W;
            end else begin
                state_d = WRITE_BACK_MEM_W;
            end
        end

        WRITE_BACK_MEM_W: begin
            // wait for responding slave to send b_valid
            if((ccu_resp_i.b_valid && ccu_req_o.b_ready)) begin
                state_d = SEND_AXI_REQ_W;
            end else begin
                state_d = WRITE_BACK_MEM_W;
            end
        end

        SEND_AXI_REQ_W: begin
            // wait for responding slave to assert aw_ready
            if(ccu_resp_i.aw_ready !='b1) begin
                state_d = SEND_AXI_REQ_W;
            end else begin
                state_d = WRITE_MEM;
            end
        end

        WRITE_MEM: begin
            // wait for responding slave to send b_valid
            if((ccu_resp_i.b_valid && ccu_req_i.b_ready)) begin
                  if(ccu_req_holder_q.aw.atop [5]) begin
                    state_d = READ_MEM;
                  end else begin
                    state_d = IDLE;
                  end
            end else begin
                state_d = WRITE_MEM;
            end
        end

        default: state_d = IDLE;


    endcase
    end

    // ----------------------
    // Output Block
    // ----------------------
    always_comb begin : ccu_output_block
        logic ar_addr_offset;

        ar_addr_offset = ccu_req_holder_q.ar.addr[3];

        // Default Assignments
        ccu_req_o  = '0;
        ccu_resp_o = '0;
        s2m_req_o  = '0;

        case(state_q)
        IDLE: begin
          ccu_resp_o.aw_ready = (!ccu_req_i.ar_valid || prio_q.waiting_w);
          ccu_resp_o.ar_ready = (!ccu_req_i.aw_valid || prio_q.waiting_r || !prio_q.waiting_w);
        end

        //---------------------
        //---- Read Branch ----
        //---------------------
        SEND_READ, SEND_INVALID_R: begin
            // send request to snooping masters
            for (int unsigned n = 0; n < NoMstPorts; n = n + 1) begin
                s2m_req_o[n].ac.addr   =   ccu_req_holder_q.ar.addr;
                s2m_req_o[n].ac.prot   =   ccu_req_holder_q.ar.prot;
                s2m_req_o[n].ac.snoop  =   (state_q == SEND_READ)   ?
                                            ccu_req_holder_q.ar.snoop :
                                            snoop_pkg::CLEAN_INVALID;
                s2m_req_o[n].ac_valid  =   !ac_handshake_holder_q[n];
            end
        end

        WAIT_RESP_R, WAIT_INVALID_W: begin
            for (int unsigned n = 0; n < NoMstPorts; n = n + 1)
              s2m_req_o[n].cr_ready  =   !cr_handshake_holder_q[n]; //'b1;
        end

        WAIT_INVALID_R: begin
            for (int unsigned n = 0; n < NoMstPorts; n = n + 1)
                s2m_req_o[n].cr_ready  =   !cr_handshake_holder_q[n]; //'b1;

            if ((cr_handshake_holder_d == '1) && (!ccu_req_holder_q.ar.lock)) begin
                ccu_resp_o.r        =   '0;
                ccu_resp_o.r.id     =   ccu_req_holder_q.ar.id;
                ccu_resp_o.r.last   =   'b1;
                ccu_resp_o.r_valid  =   'b1;
            end
        end

        READ_SNP_DATA: begin
          for (int unsigned n = 0; n < NoMstPorts; n = n + 1)
            s2m_req_o[n].cd_ready  = !cd_last_q[n] & data_available_q[n];
          // response to intiating master
          if (!r_eot) begin
            if (ccu_req_holder_q.ar.len == 0) begin
                // single data request
                logic critical_word_valid;
                critical_word_valid = ar_addr_offset ? stored_cd_data_q[0] : cd_valid_data; // TODO: check if it makes sense;
                ccu_resp_o.r.data   = cd_data_d[ar_addr_offset];
                ccu_resp_o.r.last   = critical_word_valid;
                ccu_resp_o.r_valid  = critical_word_valid;
            end else begin
                // cache line request
                ccu_resp_o.r.data  = cd_data_d[r_last];
                ccu_resp_o.r.last  = r_last;
                ccu_resp_o.r_valid = cd_valid_data;
            end
            ccu_resp_o.r.id      = ccu_req_holder_q.ar.id;
            ccu_resp_o.r.resp[3] = |shared_q;                // update if shared
            ccu_resp_o.r.resp[2] = |dirty_q;                 // update if any line dirty
          end
        end

        SEND_AXI_REQ_WRITE_BACK_R: begin
            // send writeback request
            ccu_req_o.aw_valid     = 'b1;
            ccu_req_o.aw           = '0; //default
            ccu_req_o.aw.addr      = ccu_req_holder_q.ar.addr;
            ccu_req_o.aw.addr[3:0] = 4'b0; // writeback is always full cache line
            ccu_req_o.aw.size      = 2'b11;
            ccu_req_o.aw.burst     = axi_pkg::BURST_INCR; // Use BURST_INCR for AXI regular transaction
            ccu_req_o.aw.id        = {first_responder_q, ccu_req_holder_q.ar.id[SlvAxiIDWidth-1:0]}; // It should be visible this data originates from the responder, important e.g. for AMO operations
            ccu_req_o.aw.len       = DcacheLineWords-1;
            // WRITEBACK
            ccu_req_o.aw.domain    = 2'b00;
            ccu_req_o.aw.snoop     = 3'b011;
        end

        WRITE_BACK_MEM_R: begin
          for (int unsigned n = 0; n < NoMstPorts; n = n + 1)
            s2m_req_o[n].cd_ready  = !cd_last_q[n] & data_available_q[n];
            // write data to slave (RAM)
            ccu_req_o.w_valid =  cd_valid_data;
            ccu_req_o.w.strb  =  '1;
            ccu_req_o.w.data  =   cd_data_d[w_last];
            ccu_req_o.w.last  =   w_last;
            ccu_req_o.b_ready = 'b1;
        end

        SEND_AXI_REQ_R: begin
            // forward request to slave (RAM)
            ccu_req_o.ar_valid  =   'b1;
            ccu_req_o.ar        =   ccu_req_holder_q.ar;
            ccu_req_o.r_ready   =   ccu_req_holder_q.r_ready ;
        end

        READ_MEM: begin
            // indicate slave to send data on r channel
            ccu_req_o.r_ready   =   ccu_req_i.r_ready ;
            ccu_resp_o.r        =   ccu_resp_i.r;
            ccu_resp_o.r_valid  =   ccu_resp_i.r_valid;
        end

        //---------------------
        //---- Write Branch ---
        //---------------------

        SEND_INVALID_W:begin
            for (int unsigned n = 0; n < NoMstPorts; n = n + 1) begin
                s2m_req_o[n].ac.addr  = ccu_req_holder_q.aw.addr;
                s2m_req_o[n].ac.prot  = ccu_req_holder_q.aw.prot;
                s2m_req_o[n].ac.snoop = snoop_pkg::CLEAN_INVALID;
                s2m_req_o[n].ac_valid = !ac_handshake_holder_q[n];
            end
        end

        SEND_AXI_REQ_WRITE_BACK_W: begin
            // send writeback request
            ccu_req_o.aw_valid     = 'b1;
            ccu_req_o.aw           = '0; //default
            ccu_req_o.aw.addr      = ccu_req_holder_q.aw.addr;
            ccu_req_o.aw.addr[3:0] = 4'b0; // writeback is always full cache line
            ccu_req_o.aw.size      = 2'b11;
            ccu_req_o.aw.burst     = axi_pkg::BURST_INCR; // Use BURST_INCR for AXI regular transaction
            ccu_req_o.aw.id        = {first_responder_q, ccu_req_holder_q.aw.id[SlvAxiIDWidth-1:0]}; // It should be visible this data originates from the responder, important e.g. for AMO operations
            ccu_req_o.aw.len       = DcacheLineWords-1;
            // WRITEBACK
            ccu_req_o.aw.domain    = 2'b00;
            ccu_req_o.aw.snoop     = 3'b011;
        end

        WRITE_BACK_MEM_W: begin
          for (int unsigned n = 0; n < NoMstPorts; n = n + 1)
            s2m_req_o[n].cd_ready  = !cd_last_q[n] & data_available_q[n];
          // response to intiating master
          if (!r_eot) begin
            ccu_req_o.w_valid =  cd_valid_data;
            ccu_req_o.w.strb  =  '1;
            ccu_req_o.w.data  =  cd_data_d[w_last];
            ccu_req_o.w.last  =  w_last;
            ccu_req_o.b_ready = 'b1;
          end
        end

        SEND_AXI_REQ_W: begin
            // forward request to slave (RAM)
            ccu_req_o.aw_valid  =    'b1;
            ccu_req_o.aw        =    ccu_req_holder_q.aw;
        end

        WRITE_MEM: begin
            ccu_req_o.w         =  ccu_req_i.w;
            ccu_req_o.w_valid   =  ccu_req_i.w_valid;
            ccu_req_o.b_ready   =  ccu_req_i.b_ready;

            ccu_resp_o.b        =  ccu_resp_i.b;
            ccu_resp_o.b_valid  =  ccu_resp_i.b_valid;
            ccu_resp_o.w_ready  =  ccu_resp_i.w_ready;
        end

        endcase
    end // end output block

    //-----------------------------------
    //---- Additional state registers ---
    //-----------------------------------

    // The following signal naming convention is implemented where needed:
    // *_q: the output of a register
    // *_d: the combination of the *_q signal and the respective combinational signal
    // Usually, *_d is sampled by *_q
    // *_holder_*: the signal is used to hold and/or accumulate events
    // e.g.: ccu_req_holder_q samples an incoming request from the masters
    //       ac_handshake_holder_q keep tracks of the master ports which
    //       observed an handshake on the AC channel

    // Hold incoming ACE request
    always_ff @(posedge clk_i , negedge rst_ni) begin
        if(!rst_ni) begin
            ccu_req_holder_q <= '0;
        end else if(state_q == IDLE && slv_ar_handshake) begin
            ccu_req_holder_q.ar       <=  ccu_req_i.ar;
            ccu_req_holder_q.ar_valid <=  ccu_req_i.ar_valid;
            ccu_req_holder_q.r_ready  <=  ccu_req_i.r_ready;

        end  else if(state_q == IDLE && slv_aw_handshake) begin
            ccu_req_holder_q.aw       <=  ccu_req_i.aw;
            ccu_req_holder_q.aw_valid <=  ccu_req_i.aw_valid;
        end
    end

    // Hold snoop AC handshake
    always_ff @ (posedge clk_i, negedge rst_ni) begin
      if(!rst_ni) begin
        ac_handshake_holder_q <= '0;
      end else if(state_q == IDLE && (slv_ar_handshake || slv_aw_handshake)) begin
        ac_handshake_holder_q <= initiator_d;
      end else if(state_q == SEND_READ || state_q == SEND_INVALID_R || state_q == SEND_INVALID_W) begin
        ac_handshake_holder_q <= ac_handshake_holder_d;
      end else begin
        ac_handshake_holder_q <= '0;
      end
    end

    assign ac_handshake_holder_d = ac_handshake | ac_handshake_holder_q;

    // Hold snoop CR
    always_ff @ (posedge clk_i, negedge rst_ni) begin
      logic snoop_resp_found;
      if(!rst_ni) begin
        cr_handshake_holder_q <= '0;
        data_available_q      <= '0;
        shared_q              <= '0;
        dirty_q               <= '0;
        response_error_q      <= '0;
        first_responder_q     <= '0;
        snoop_resp_found      <= 1'b0;
      end else if(state_q == IDLE) begin
        cr_handshake_holder_q <= '0;
        data_available_q      <= '0;
        shared_q              <= '0;
        dirty_q               <= '0;
        response_error_q      <= '0;
        first_responder_q     <= '0;
        snoop_resp_found      <= 1'b0;
      end else if(state_q == SEND_READ || state_q == SEND_INVALID_R || state_q == SEND_INVALID_W) begin
        cr_handshake_holder_q <= initiator_q;
      end else begin
        for (int i = 0; i < NoMstPorts; i = i + 1) begin
          if(cr_handshake[i]) begin
            cr_handshake_holder_q[i] <=   1'b1; // if cr_handshake[i] = 1'b1 then cr_handshake_holder_d[i] = 1'b1
            data_available_q[i]      <=   data_available_d[i];
            shared_q[i]              <=   shared_d[i];
            dirty_q[i]               <=   dirty_d[i];
            response_error_q[i]      <=   response_error_d[i];
          end
        end
        if (!snoop_resp_found) begin
          for (int i = 0; i < NoMstPorts; i = i + 1) begin
            if(cr_handshake[i] & data_available_d[i] & !response_error_d[i]) begin
              first_responder_q <= i[MstIdxBits-1:0];
              snoop_resp_found <= 1'b1;
              break;
            end
          end
        end
      end
    end

    assign cr_handshake_holder_d =  cr_handshake | cr_handshake_holder_q;
    for (genvar i = 0; i < NoMstPorts; i = i + 1) begin
        assign data_available_d[i]      =  m2s_resp_i[i].cr_resp.dataTransfer;
        assign shared_d[i]              =  m2s_resp_i[i].cr_resp.isShared;
        assign dirty_d[i]               =  m2s_resp_i[i].cr_resp.passDirty;
        assign response_error_d[i]      =  m2s_resp_i[i].cr_resp.error;
    end

    // Hold snoop CD
    always_ff @ (posedge clk_i, negedge rst_ni) begin
      if(!rst_ni) begin
        data_received_q    <= '0;
        cd_last_q          <= '0;
        m2s_resp_holder_q  <= '0;
        cd_data_q          <= '0;
        stored_cd_data_q   <= '0;
      end else begin
        if(state_q == IDLE) begin
          data_received_q   <= '0;
          m2s_resp_holder_q <= '0;
          cd_last_q         <= '0;
          cd_data_q         <= '0;
          stored_cd_data_q  <= '0;
        end else begin
          for (int i = 0; i < NoMstPorts; i = i + 1) begin
            if (state_q == READ_SNP_DATA) begin
              if(m2s_resp_i[i].cd_valid) begin
                data_received_q[i]    <= 1'b1;
                cd_last_q[i]          <= cd_last_d;
                m2s_resp_holder_q[i]  <= m2s_resp_i[i];
              end
              if (data_received_q[i] & ccu_resp_o.r_valid) begin
                data_received_q[i] <= '0;
                m2s_resp_holder_q  <= '0;
              end
              if (responder_cd_handshake) begin
                cd_data_q[responder_resp.cd.last] <= responder_resp.cd.data;
              end
              if (responder_cd_handshake & !slv_r_handshake) begin
                stored_cd_data_q <= stored_cd_data_q + 1;
              end else if(slv_r_handshake & !responder_cd_handshake) begin
                stored_cd_data_q <= stored_cd_data_q - 1;
              end
            end else if (state_q == WRITE_BACK_MEM_R || state_q == WRITE_BACK_MEM_W) begin
              if(m2s_resp_i[i].cd_valid) begin
                data_received_q[i]    <= 1'b1;
                cd_last_q[i]          <= cd_last_d;
                m2s_resp_holder_q[i]  <= m2s_resp_i[i];
              end
              if (data_received_q[i] & ccu_req_o.w_valid) begin
                data_received_q[i] <= '0;
                m2s_resp_holder_q  <= '0;
              end
              if (responder_cd_handshake) begin
                cd_data_q[responder_resp.cd.last] <= responder_resp.cd.data;
              end
              if (responder_cd_handshake & !mst_w_handshake) begin
                stored_cd_data_q <= stored_cd_data_q + 1;
              end else if(mst_w_handshake & !responder_cd_handshake) begin
                stored_cd_data_q <= stored_cd_data_q - 1;
              end
            end
          end
        end
      end
    end

  for (genvar i = 0; i < NoMstPorts; i = i + 1) begin
    assign cd_last_d[i] = cd_last_q[i] | (m2s_resp_i[i].cd.last & data_available_q[i]);
    assign data_received_d[i] = m2s_resp_i[i].cd_valid | data_received_q;
  end
  for (genvar i = 0; i < DcacheLineWords; i = i + 1) begin
    assign cd_data_d[i] = responder_resp.cd.last == i ? responder_resp.cd.data : cd_data_q;
  end

  always_ff @ (posedge clk_i, negedge rst_ni) begin
    if(!rst_ni) begin
      r_last <= 1'b0;
      r_eot  <= 1'b0;
    end else begin
      if(state_q == IDLE) begin
        r_last <= 1'b0;
        r_eot  <= 1'b0;
      end else if (slv_r_handshake) begin
        r_last <= !r_last;
        if (ccu_resp_o.r.last)
          r_eot <= 1'b1;
      end
    end
  end

  always_ff @ (posedge clk_i, negedge rst_ni) begin
    if(!rst_ni) begin
      w_last <= 1'b0;
      w_eot  <= 1'b0;
    end else begin
      if(state_q == IDLE) begin
        w_last <= 1'b0;
        w_eot  <= 1'b0;
      end else if (mst_w_handshake) begin
        w_last <= !w_last;
        if (w_last)
          w_eot <= 1'b1;
      end
    end
  end

  `ifndef VERILATOR
  // pragma translate_off
  initial begin
    a_dcache_line_words : assert (DcacheLineWords == 2) else
        $error("The ccu_fsm module is currently hardcoded to only support DcacheLineWidth = 2 * AxiDataWidth");
  end
  // pragma translate_on
  `endif



endmodule
