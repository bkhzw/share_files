`ifndef __PCIE_PHY_JTAG_BERT_TEST_SV__
 `define __PCIE_PHY_JTAG_BERT_TEST_SV__
// LANEN_DIG_TX_LBERT_CTL 0x1(N*2+0)71  bit15 reserved bit14:5 PAT0 bit4 trigger_error bit3:0 MODE
// LANEN_DIG_RX_LBERT_CTL 0x1(N*2+0)93  bit15:11 reserved bit10 error_count_clr_n bit9 use_pat_sel bit8 use_sample_cnt bit7 ber_sel bit6:5 pat_sel bit4 sync bit3:0 MODE

class pcie_phy_jtag_bert_test extends pcie_base_test;
    pcie_jtag_base_seq jtag_api[`MAX_NUM_PHYS];

    `uvm_component_utils(pcie_phy_jtag_bert_test)

    function new(string name = "pcie_phy_jtag_bert_test", uvm_component parent);
        super.new(name, parent);
    endfunction

    virtual task reset_phase(uvm_phase phase);
        super.reset_phase(phase);
        `uvm_info(get_type_name(), "Enter reset_phase...", UVM_LOW)
        phase.raise_objection(this);
        int_vif.bd_jtag_o_tdo_en = 0;
        int_vif.bd_jtag_o_tdo = 0;
        phase.drop_objection(this);
        `uvm_info(get_type_name(), "Exit reset_phase...", UVM_LOW)
    endtask

    virtual task configure_phase(uvm_phase phase);
        phase.raise_objection(this);

        phase.drop_objection(this);
        `uvm_info(get_type_name(), "Exit configure_phase...", UVM_LOW)
    endtask

    virtual task post_configure_phase(uvm_phase phase);
        phase.raise_objection(this);

        phase.drop_objection(this);
        `uvm_info(get_type_name(), "Exit post_configure_phase...", UVM_LOW)
    endtask

    virtual task do_main_task();
        bit[15:0]    read_data;
        bit[15:0]    write_data;
        uvm_status_e status;
        int          pn;
        int          jtag_sel;

        `uvm_info(get_type_name(), "Enter do_main_task...", UVM_LOW)
        if(!$value$plusargs("jtag_sel=%d", jtag_sel)) begin
            jtag_sel = $urandom_range(1, 4);
        end
       `uvm_info("dbg_info", $sformatf("jtag_sel = %0d, sel phy#%0d", jtag_sel, jtag_sel-1), UVM_LOW)

        //===========================================
        // clk_gen for TDR_sel
        //===========================================
        fork
            forever #10ns int_vif.SPI_0_DO_0<=~int_vif.SPI_0_DO_0;
        join_none
        
        //===========================================
        // disable_checks, phy_link unuse in dft test
        //===========================================
        void'(env.pcie_ss_env.pcie_vip_agt.pcie_agent.err_check.disable_checks("ACTIVE_PL","POLLING_ACTIVE","phy_polling_active_timeout"));

        //sys_cfg_vif.pcie_vip_reset = 1; //hold pcie vip reset
        void'(uvm_hdl_force({`DV_STRINGIFY(`PCIE_TB_TOP),$sformatf(".reset")}, 1)); //hold pcie vip reset

        //ral.pcie_top.pcie_ss_clkrst_reg_0.sw_lane_reset_n.write(status,0); //all phy lane rst hold

        //===========================================
        // if foreach 4 phy, case_time too long about 450Wns/phy. so split case and use jtag_sel sel phy
        //===========================================
        //for(int pn=0; pn<`MAX_NUM_PHYS; pn++) begin
        begin
            case(jtag_sel)
                1: pn = 0;
                2: pn = 1;
                3: pn = 2;
                4: pn = 3;
                default: `uvm_fatal("dbg_info", $sformatf("bad jtag_sel=%0d!", jtag_sel))
            endcase
            `uvm_info("dbg_info", $sformatf("begin test phy%0d", pn), UVM_LOW)
            jtag_tdr_driver(pn+1);
            //uvm_hdl_force({`DV_STRINGIFY(`PCIE_TOP),$sformatf(".pcie_core_0.pcie_phy_pipe_0.phy%0d_cr_para_sel",pn)},2'b00);
            write_ir(pn,(8'h31)<<(`SVT_JTAG_MAX_INSTRUCTION_WIDTH-8)); //pcie use 8 bit 
            `uvm_info(get_full_name(),$sformatf("write ir done"),UVM_LOW)

            // wait sram_init_done = 1
            while(1) begin
                jtag_cr_read(pn,'h197, read_data);
                `uvm_info("dbg_info", $sformatf("read %0h @SRAM_OUT", read_data), UVM_LOW)
                if(read_data[0] == 1) break;
            end
            // write sram_ext_ld_done
            jtag_cr_read(pn,'h195, read_data);
            write_data = read_data | (2'b11);
            jtag_cr_write(pn, 'h195, write_data);
            `uvm_info("dbg_info", $sformatf("read %0h & write %0h @SRAM_OVRD_IN", read_data, write_data), UVM_LOW)
            // read sram_ext_ld_done @RAWCMN_DIG_AON_SRAM_IN
            jtag_cr_read(pn,'h196, read_data);
            `uvm_info("dbg_info", $sformatf("read %0h @SRAM_IN", read_data), UVM_LOW)

            jtag_cr_write(pn, 'h6001, 'h0003); // RAWLANEN_DIG_PCS_XF_TX_OVRD_IN_1 boardcast
            jtag_cr_write(pn, 'h6006, 'h0003); // RAWLANEN_DIG_PCS_XF_RX_OVRD_IN_1

            ate_initialize(pn);
            bert_test(pn);
        end

        `uvm_info(get_type_name(), "Exit do_main_task...", UVM_LOW)
    endtask

    task write_ir(bit[1:0] phy_num, bit [`SVT_JTAG_MAX_INSTRUCTION_WIDTH-1:0] cmd);
        pcie_jtag_base_seq jtag_seq;
        
        jtag_seq = pcie_jtag_base_seq::type_id::create("jtag_seq");
        jtag_seq.cmd_type = svt_jtag_types::IR;
        jtag_seq.cmd_ir = cmd;
        //jtag_seq.start(vsqr.jtag_vip_sqr[phy_num]);
        jtag_seq.start(soc_vsqr.pcie_jtag_vip_sqr);
    endtask

    task shift_dr(bit[1:0] phy_num, input bit [`SVT_JTAG_MAX_DATA_WIDTH-1:0] cmd, int cmd_len = 18, output bit[`SVT_JTAG_MAX_DATA_WIDTH-1:0] sampled_tdo);
        pcie_jtag_base_seq jtag_seq;
        
        `uvm_info(get_full_name(),$sformatf("cmd - 0x%0x",cmd),UVM_LOW)
        jtag_seq = pcie_jtag_base_seq::type_id::create("jtag_seq");
        jtag_seq.cmd_type = svt_jtag_types::DR;
        jtag_seq.cmd_dr = cmd;
        jtag_seq.cmd_len = cmd_len;
        //jtag_seq.start(vsqr.jtag_vip_sqr[phy_num]);
        jtag_seq.start(soc_vsqr.pcie_jtag_vip_sqr);
        sampled_tdo = jtag_seq.sampled_tdo;
    endtask
    
    task jtag_cr_write(bit[1:0] phy_num, bit[15:0] addr, bit[15:0] data);
        bit[17:0] cmd;
        bit [`SVT_JTAG_MAX_DATA_WIDTH-1:0] unused;

        cmd = {2'b00,addr};
        shift_dr(phy_num,cmd,18,unused);
        cmd = {2'b01,data};
        shift_dr(phy_num,cmd,18,unused);
    endtask

    task jtag_cr_read(bit[1:0] phy_num, bit[15:0] addr, output bit[15:0] data);
        bit[17:0] cmd;
        bit [`SVT_JTAG_MAX_DATA_WIDTH-1:0] unused;

        cmd = {2'b11,addr};
        shift_dr(phy_num,cmd,18,unused);
        cmd = {2'b10,16'h0};
        shift_dr(phy_num,cmd,18,data);
    endtask

    task ate_initialize(bit[1:0] pn);
        bit[15:0] read_data;

        jtag_cr_read(pn, 'h0197, read_data);
        if(read_data[0] == 1)
          `uvm_info(get_full_name(),$sformatf("PHY%0d Expecting SRAM_INIT_DONE to be asserted",pn),UVM_LOW)
        else 
          `uvm_error(get_full_name(),$sformatf("PHY%0d Expecting SRAM_INIT_DONE should be asserted",pn))
        `uvm_info(get_full_name(),$sformatf("PHY%0d Overriding sram_ext_ld_done to 1'b1",pn),UVM_LOW)
        jtag_cr_write(pn, 'h174, 'h330); //en refa/b clk_en
        jtag_cr_write(pn, 'h2, 'h3); //en refa/b clk_en
        jtag_cr_write(pn, 'h3, 'h3); //en refa/b clk_en
        jtag_cr_write(pn, 'h0195, 'h0003);

        //$display ("\t\t update firmware if necessary");
        //vec_pause (2);
        //$display ("\t\t Insert the vector pause for FW updates if necessary in ate.vec at line number %d", tb.vec_count);
        //vec_pause (2); 

        //from ate initialize
        //load_cfg(pn,{getenv("PCIE_TOP_VV_DIR"),"/test/testcases/pcie_phy_jtag_bert_test/ate_initialize_test.txt"});
        load_cfg(pn,{getenv("SOC_VV_DIR"),"/test/testcases/sub_sys/pcie/pcie_phy_jtag_bert_test/ate_initialize_test.txt"});

        `uvm_info(get_full_name(),$sformatf("PHY%0d Start ATE test sequence 2 , PHY power up",pn),UVM_LOW)

        `uvm_info(get_full_name(),$sformatf("PHY%0d JTAG ID READ for selective register read enabling",pn),UVM_LOW)
        jtag_cr_read(pn, 'h0000, read_data);
        if(read_data == 'h74cd)
            `uvm_info(get_full_name(),$sformatf("PHY%0d JTAG ID is read as expected",pn),UVM_LOW)
        else
            `uvm_error(get_full_name(),$sformatf("PHY%0d JTAG ID is not as expected,read data is 0x%0x",pn,read_data))
        jtag_cr_read(pn, 'h0001, read_data);
        if(read_data == 'h6d4b)
            `uvm_info(get_full_name(),$sformatf("PHY%0d JTAG ID is read as expected",pn),UVM_LOW)
        else
            `uvm_error(get_full_name(),$sformatf("PHY%0d JTAG ID is not as expected,read data is 0x%0x",pn,read_data))

        //load_cfg(pn,{getenv("PCIE_TOP_VV_DIR"),"/test/testcases/pcie_phy_jtag_bert_test/ate_power_up.txt"});
        load_cfg(pn,{getenv("SOC_VV_DIR"),"/test/testcases/sub_sys/pcie/pcie_phy_jtag_bert_test/ate_power_up.txt"});

        //wait_signal({`DV_STRINGIFY(`PCIE_UPCS),".phy0.pcs_raw.aon.aon_lane0.lane_init_pwrup_done_r"}, 1'b1);
        //wait_signal({`DV_STRINGIFY(`PCIE_UPCS),".phy0.pma.dig.lane0.asic_xface.rx_ack_i"}, 1'b0);
        //wait_signal({`DV_STRINGIFY(`PCIE_UPCS),".phy0.pma.dig.lane0.asic_xface.rx_valid_i"}, 1'b0);
        //wait_signal({`DV_STRINGIFY(`PCIE_UPCS),".phy0.pcs_raw.lane0.lane_pcs_xface.rx_ack_i"}, 1'b0);
        //wait_signal({`DV_STRINGIFY(`PCIE_UPCS),".phy0.pma.dig.lane0.asic_xface.tx_ack_i"}, 1'b0);
        //wait_signal({`DV_STRINGIFY(`PCIE_UPCS),".phy0.pcs_raw.lane0.lane_pcs_xface.tx_ack_i"}, 1'b0);

        //wait_signal({`DV_STRINGIFY(`PCIE_UPCS),".phy0.pcs_raw.aon.aon_lane1.lane_init_pwrup_done_r"}, 1'b1);
        //wait_signal({`DV_STRINGIFY(`PCIE_UPCS),".phy0.pma.dig.lane1.asic_xface.rx_ack_i"}, 1'b0);
        //wait_signal({`DV_STRINGIFY(`PCIE_UPCS),".phy0.pma.dig.lane1.asic_xface.rx_valid_i"}, 1'b0);
        //wait_signal({`DV_STRINGIFY(`PCIE_UPCS),".phy0.pcs_raw.lane1.lane_pcs_xface.rx_ack_i"}, 1'b0);
        //wait_signal({`DV_STRINGIFY(`PCIE_UPCS),".phy0.pma.dig.lane1.asic_xface.tx_ack_i"}, 1'b0);
        //wait_signal({`DV_STRINGIFY(`PCIE_UPCS),".phy0.pcs_raw.lane1.lane_pcs_xface.tx_ack_i"}, 1'b0);

        //wait_signal({`DV_STRINGIFY(`PCIE_UPCS),".phy0.pcs_raw.aon.aon_lane2.lane_init_pwrup_done_r"}, 1'b1);
        //wait_signal({`DV_STRINGIFY(`PCIE_UPCS),".phy0.pma.dig.lane2.asic_xface.rx_ack_i"}, 1'b0);
        //wait_signal({`DV_STRINGIFY(`PCIE_UPCS),".phy0.pma.dig.lane2.asic_xface.rx_valid_i"}, 1'b0);
        //wait_signal({`DV_STRINGIFY(`PCIE_UPCS),".phy0.pcs_raw.lane2.lane_pcs_xface.rx_ack_i"}, 1'b0);
        //wait_signal({`DV_STRINGIFY(`PCIE_UPCS),".phy0.pma.dig.lane2.asic_xface.tx_ack_i"}, 1'b0);
        //wait_signal({`DV_STRINGIFY(`PCIE_UPCS),".phy0.pcs_raw.lane2.lane_pcs_xface.tx_ack_i"}, 1'b0);

        //wait_signal({`DV_STRINGIFY(`PCIE_UPCS),".phy0.pcs_raw.aon.aon_lane3.lane_init_pwrup_done_r"}, 1'b1);
        //wait_signal({`DV_STRINGIFY(`PCIE_UPCS),".phy0.pma.dig.lane3.asic_xface.rx_ack_i"}, 1'b0);
        //wait_signal({`DV_STRINGIFY(`PCIE_UPCS),".phy0.pma.dig.lane3.asic_xface.rx_valid_i"}, 1'b0);
        //wait_signal({`DV_STRINGIFY(`PCIE_UPCS),".phy0.pcs_raw.lane3.lane_pcs_xface.rx_ack_i"}, 1'b0);
        //wait_signal({`DV_STRINGIFY(`PCIE_UPCS),".phy0.pma.dig.lane3.asic_xface.tx_ack_i"}, 1'b0);
        //wait_signal({`DV_STRINGIFY(`PCIE_UPCS),".phy0.pcs_raw.lane3.lane_pcs_xface.tx_ack_i"}, 1'b0);

        //Insert dummy cycle     148000 for normal FW in ate.vec at line number                17968
        `uvm_info(get_full_name(),$sformatf("PHY%0d Waiting for        300 microseconds for expected events to take place",pn),UVM_LOW)
        #300us;
        `uvm_info(get_full_name(),$sformatf("PHY%0d Reading INIT_PWRUP_DONE on all lanes",pn),UVM_LOW)
        for(int ln=0; ln<4; ln++) begin
            jtag_cr_read(pn, 'h3014+ln*'h200, read_data);
            if(read_data == 'h1)
                `uvm_info(get_full_name(),$sformatf("PHY%0d INIT_PWRUP_DONE(bit[0]) on Lane %0d to be asserted",pn,ln),UVM_LOW)
            else
                `uvm_error(get_full_name(),$sformatf("PHY%0d INIT_PWRUP_DONE(bit[0]) on Lane %0d de-asserted, read_data is 0x%0x",pn,ln,read_data))
        end

        `uvm_info(get_full_name(),$sformatf("PHY%0d Reading RX ACK and RX VALID at PMA level",pn),UVM_LOW)
        for(int ln=0; ln<4; ln++) begin
            jtag_cr_read(pn, 'h103c+ln*'h200, read_data);
            //if(read_data == 'h70)
            if(read_data[1:0] == 'h0)
                `uvm_info(get_full_name(),$sformatf("PHY%0d Expecting RX ACK(bit[0]) to be de-asserted and RX VALID(bit[1]) to be de-asserted on Channel%0d",pn,ln),UVM_LOW)
            else
                `uvm_error(get_full_name(),$sformatf("PHY%0d Expecting RX ACK(bit[0]) to be de-asserted and RX VALID(bit[1]) to be de-asserted on Channel%0d,read data is 0x%0x",pn,ln,read_data))
        end

        `uvm_info(get_full_name(),$sformatf("PHY%0d Reading RX ACK at PHY level",pn),UVM_LOW)
        for(int ln=0; ln<4; ln++) begin
            jtag_cr_read(pn, 'h2019+ln*'h200, read_data);
            if(read_data == 'h0)
                `uvm_info(get_full_name(),$sformatf("PHY%0d Expecting RX ACK(bit[0]) to be de-asserted on Channle%0d",pn,ln),UVM_LOW)
            else
                `uvm_error(get_full_name(),$sformatf("PHY%0d Expecting RX ACK(bit[0]) to be de-asserted on Channle%0d,read data is 0x%0x",pn,ln,read_data))
        end

        `uvm_info(get_full_name(),$sformatf("PHY%0d Reading TX ACK at PMA level",pn),UVM_LOW)
        for(int ln=0; ln<4; ln++) begin
            jtag_cr_read(pn, 'h1012+ln*'h200, read_data);
            if(read_data == 'h0)
                `uvm_info(get_full_name(),$sformatf("PHY%0d Expecting TX ACK(bit[0]) of Channel%0d to be de-asserted",pn,ln),UVM_LOW)
            else
                `uvm_error(get_full_name(),$sformatf("PHY%0d Expecting TX ACK(bit[0]) of Channel%0d to be de-asserted,read data is 0x%0x",pn,ln,read_data))
        end

        `uvm_info(get_full_name(),$sformatf("PHY%0d Reading TX ACK at PHY level",pn),UVM_LOW)
        for(int ln=0; ln<4; ln++) begin
            jtag_cr_read(pn, 'h2004+ln*'h200, read_data);
            if(read_data == 'h0)
                `uvm_info(get_full_name(),$sformatf("PHY%0d Expecting TX ACK(bit[0]) of Channel%0d to be de-asserted",pn,ln),UVM_LOW)
            else
                `uvm_error(get_full_name(),$sformatf("PHY%0d Expecting TX ACK(bit[0]) of Channel%0d to be de-asserted,read data is 0x%0x",pn,ln,read_data))
        end
        `uvm_info(get_full_name(),$sformatf("PHY%0d End of ATE test sequence           2",pn),UVM_LOW)
        `uvm_info(get_full_name(),$sformatf("PHY%0d Start ATE test sequence           3 , Check pll states",pn),UVM_LOW)

        `uvm_info(get_full_name(),$sformatf("PHY%0d Read mpll and ropll state",pn),UVM_LOW)
        for(int ln=0; ln<4; ln++) begin
            jtag_cr_read(pn, 'h20a5+ln*'h200, read_data);
            if(read_data == 'h2)
                `uvm_info(get_full_name(),$sformatf("PHY%0d Lane %0d ROPLL state",pn,ln),UVM_LOW)
            else
                `uvm_error(get_full_name(),$sformatf("PHY%0d Lane %0d ROPLL state,read data is 0x%0x",pn,ln,read_data))
            jtag_cr_read(pn, 'h20a3+ln*'h200, read_data);
            if(read_data == 'h2)
                `uvm_info(get_full_name(),$sformatf("PHY%0d Lane %0d MPLLB state bit[1]",pn,ln),UVM_LOW)
            else
                `uvm_error(get_full_name(),$sformatf("PHY%0d Lane %0d MPLLB state bit[1],read data is 0x%0x",pn,ln,read_data))
        end

        `uvm_info(get_full_name(),$sformatf("PHY%0d End of ATE test sequence           3",pn),UVM_LOW)
        `uvm_info(get_full_name(),$sformatf("PHY%0d Start ATE test sequence           4 , set loopback_mode 00",pn),UVM_LOW)
        `uvm_info(get_full_name(),$sformatf("PHY%0d Overriding lane_rx2tx_par_lb_en and lane_tx2rx_ser_lb_en",pn),UVM_LOW)
        if(!$test$plusargs("inter_lb")) begin
            jtag_cr_write(pn, 'h6028, 'h000a);
        end
        else begin
            jtag_cr_write(pn, 'h6028, 'h000e);
        end
        `uvm_info(get_full_name(),$sformatf("PHY%0d End of ATE test sequence           4",pn),UVM_LOW)
        `uvm_info(get_full_name(),$sformatf("PHY%0d Start ATE test sequence           5 , rx adaptation",pn),UVM_LOW)
        `uvm_info(get_full_name(),$sformatf("PHY%0d Enabling TX LBERT in LFSR31",pn),UVM_LOW)
        jtag_cr_write(pn, 'h5071, 'h0001);
        `uvm_info(get_full_name(),$sformatf("PHY%0d Toggle rx resets",pn),UVM_LOW)
        jtag_cr_write(pn, 'h6020, 'h80ab);
        jtag_cr_write(pn, 'h6020, 'h80ab);
        `uvm_info(get_full_name(),$sformatf("PHY%0d Waiting for         10 microseconds for expected events to take place",pn),UVM_LOW)
        #10us;
        for(int ln=0; ln<4; ln++) begin
            jtag_cr_read(pn, 'h103c+ln*'h200, read_data);
            //if(read_data == 'h71)
            if(read_data[0] == 'h1)
                `uvm_info(get_full_name(),$sformatf("PHY%0d Expecting RX ACK(bit[0]) to be asserted on Channel%0d",pn,ln),UVM_LOW)
            else
                `uvm_error(get_full_name(),$sformatf("PHY%0d Expecting RX ACK(bit[0]) to be asserted on Channel%0d,read data is 0x%0x",pn,ln,read_data))
        end
        jtag_cr_write(pn, 'h6020, 'h80aa);
        jtag_cr_write(pn, 'h6020, 'h80aa);
        `uvm_info(get_full_name(),$sformatf("PHY%0d Waiting for         100 microseconds for expected events to take place",pn),UVM_LOW)
        #100us;
        for(int ln=0; ln<4; ln++) begin
            jtag_cr_read(pn, 'h103c+ln*'h200, read_data);
            //if(read_data == 'h74)
            if(read_data[0] == 'h0)
                `uvm_info(get_full_name(),$sformatf("PHY%0d Expecting RX ACK(bit[0]) to be de-asserted on Channel%0d",pn,ln),UVM_LOW)
            else
                `uvm_error(get_full_name(),$sformatf("PHY%0d Expecting RX ACK(bit[0]) to be de-asserted on Channel%0d,read data is 0x%0x",pn,ln,read_data))
        end
        `uvm_info(get_full_name(),$sformatf("PHY%0d Waiting for         60 microseconds for expected events to take place",pn),UVM_LOW)
        #60us;
        `uvm_info(get_full_name(),$sformatf("PHY%0d Enable rxX_data_en",pn),UVM_LOW)
        jtag_cr_write(pn, 'h6020, 'hc0aa);
        `uvm_info(get_full_name(),$sformatf("PHY%0d Waiting for         50 microseconds for expected events to take place",pn),UVM_LOW)
        #50us;
        for(int ln=0; ln<4; ln++) begin
            jtag_cr_read(pn, 'h103c+ln*'h200, read_data);
            if(read_data[1:0] == 'b10)
                `uvm_info(get_full_name(),$sformatf("PHY%0d Expecting RX ACK(bit[0]) to be de-asserted and RX VALID(bit[1]) to be asserted on Channel%0d",pn,ln),UVM_LOW)
            else
                `uvm_error(get_full_name(),$sformatf("PHY%0d Expecting RX ACK(bit[0]) to be de-asserted and RX VALID(bit[1]) to be asserted on Channel%0d,read data is 0x%0x",pn,ln,read_data))
        end
        `uvm_info(get_full_name(),$sformatf("PHY%0d Overriding rx_adapt_req",pn),UVM_LOW)
        jtag_cr_write(pn, 'h600a, 'h3);
        jtag_cr_write(pn, 'h600a, 'h3);
        `uvm_info(get_full_name(),$sformatf("PHY%0d Waiting for rx adaptation to complete...",pn),UVM_LOW)
        //`uvm_info(get_full_name(),"Waiting for        100 microseconds for expected events to take place",UVM_LOW)
        //`uvm_info(get_full_name(),"Insert dummy cycle      49000 for normal FW in ate.vec at line number                27356",UVM_LOW)
        for(int ln=0; ln<4; ln++) begin
            wait_signal({`DV_STRINGIFY(`PCIE_UPCS),$sformatf(".phy%0d.pcs_raw.lane%0d.lane_pcs_xface.rx_adapt_ack_r",pn,ln)}, 1'b1);
        end
        #100us;
        `uvm_info(get_full_name(),$sformatf("PHY%0d Reading RX ADAPT ACK at PHY level",pn),UVM_LOW)
        for(int ln=0; ln<4; ln++) begin
            jtag_cr_read(pn, 'h201a+ln*'h200, read_data);
            if(read_data == 'h1)
                `uvm_info(get_full_name(),$sformatf("PHY%0d Expecting RX ADAPT ACK(bit[0]) to be asserted on Channle%0d",pn,ln),UVM_LOW)
            else
                `uvm_error(get_full_name(),$sformatf("PHY%0d Expecting RX ADAPT ACK(bit[0]) to be asserted on Channle%0d,read data is 0x%0x",pn,ln,read_data))
        end
        jtag_cr_write(pn, 'h600a, 'h2);
        jtag_cr_write(pn, 'h600a, 'h2);
        `uvm_info(get_full_name(),$sformatf("PHY%0d Waiting for        100 microseconds for expected events to take place",pn),UVM_LOW)
        #100us;
        `uvm_info(get_full_name(),$sformatf("PHY%0d Reading RX ADAPT ACK at PHY level",pn),UVM_LOW)
        for(int ln=0; ln<4; ln++) begin
            jtag_cr_read(pn, 'h201a+ln*'h200, read_data);
            if(read_data == 'h0)
                `uvm_info(get_full_name(),$sformatf("PHY%0d Expecting RX ADAPT ACK(bit[0]) to be de_asserted on Channle%0d",pn,ln),UVM_LOW)
            else
                `uvm_error(get_full_name(),$sformatf("PHY%0d Expecting RX ADAPT ACK(bit[0]) to be de_asserted on Channle%0d,read data is 0x%0x",pn,ln,read_data))
        end
        `uvm_info(get_full_name(),$sformatf("PHY%0d Disable rxX_data_en after adaptation done",pn),UVM_LOW)
        jtag_cr_write(pn, 'h6020, 'h80aa);
        `uvm_info(get_full_name(),$sformatf("PHY%0d Disable TX LBERT in LFSR31",pn),UVM_LOW)
        jtag_cr_write(pn, 'h5071, 'h0);
        `uvm_info(get_full_name(),$sformatf("PHY%0d End of ATE test sequence           5",pn),UVM_LOW)

    endtask

    task bert_test(bit[1:0] pn);
        bit[15:0] read_data;
        bit [3:0] bert_mode;
        bit [31:0] Pattern;
        bit        trigger_error = 1'b1;
        uvm_status_e        status;
        uvm_reg             pcie_phy_reg;

        `uvm_info(get_full_name(),$sformatf("PHY%0d Start ATE test sequence           6 , run bert test",pn),UVM_LOW)
        `uvm_info(get_full_name(),$sformatf("PHY%0d BERT_TEST",pn),UVM_LOW)
        `uvm_info(get_full_name(),$sformatf("PHY%0d Enabling TX LBERT in LFSR31",pn),UVM_LOW)
        jtag_cr_write(pn, 'h5071, 'h1);
        `uvm_info(get_full_name(),$sformatf("PHY%0d Re-enable rxX_data_en to track receive data",pn),UVM_LOW)
        jtag_cr_write(pn, 'h6020, 'hc0aa);
        `uvm_info(get_full_name(),$sformatf("PHY%0d Waiting for         50 microseconds for expected events to take place",pn),UVM_LOW)
        #50us;
        `uvm_info(get_full_name(),$sformatf("PHY%0d Reading RX ACK and RX VALID at PMA level",pn),UVM_LOW)
        for(int ln=0; ln<4; ln++) begin
            jtag_cr_read(pn, 'h103c+ln*'h200, read_data);
            //if(read_data == 'h76)
            if(read_data[1:0] == 'h2)
                `uvm_info(get_full_name(),$sformatf("PHY%0d Expecting RX ACK(bit[0]) to be de-asserted and RX VALID(bit[1]) to be asserted on Channel%0d",pn,ln),UVM_LOW)
            else
                `uvm_error(get_full_name(),$sformatf("PHY%0d Expecting RX ACK(bit[0]) to be de-asserted and RX VALID(bit[1]) to be asserted on Channel%0d,read data is 0x%0x",pn,ln,read_data))
        end
        `uvm_info(get_full_name(),$sformatf("PHY%0d Reading RX ACK and RX VALID at PMA level",pn),UVM_LOW)
        for(int ln=0; ln<4; ln++) begin
            jtag_cr_read(pn, 'h20a5+ln*'h200, read_data);
            if(read_data == 'h2)
                `uvm_info(get_full_name(),$sformatf("PHY%0d Read tx ropll state on Channel%0d",pn,ln),UVM_LOW)
            else
                `uvm_error(get_full_name(),$sformatf("PHY%0d Read tx ropll state on Channel%0d,read data is 0x%0x",pn,ln,read_data))
        end
        `uvm_info(get_full_name(),$sformatf("PHY%0d Enabling RX LBERT in LFSR31 and setting RX LBERT SYNC to 1",pn),UVM_LOW)
        jtag_cr_write(pn, 'h5093, 'h1);
        `uvm_info(get_full_name(),$sformatf("PHY%0d Set RX LBERT SYNC to 0",pn),UVM_LOW)
        jtag_cr_write(pn, 'h5093, 'h1);
        `uvm_info(get_full_name(),$sformatf("PHY%0d Set RX LBERT SYNC to 1",pn),UVM_LOW)
        jtag_cr_write(pn, 'h5093, 'h11);
        `uvm_info(get_full_name(),$sformatf("PHY%0d Set RX LBERT SYNC to 0",pn),UVM_LOW)
        jtag_cr_write(pn, 'h5093, 'h1);
        `uvm_info(get_full_name(),$sformatf("PHY%0d Set RX LBERT SYNC to 1",pn),UVM_LOW)
        jtag_cr_write(pn, 'h5093, 'h11);
        `uvm_info(get_full_name(),$sformatf("PHY%0d Set RX LBERT SYNC to 0",pn),UVM_LOW)
        jtag_cr_write(pn, 'h5093, 'h1);
        `uvm_info(get_full_name(),$sformatf("PHY%0d BERT_TEST",pn),UVM_LOW)
        `uvm_info(get_full_name(),$sformatf("PHY%0d Waiting for         10 microseconds for expected events to take place",pn),UVM_LOW)
        #10us;
        for(int ln=0; ln<4; ln++) begin
            jtag_cr_read(pn, 'h1094+ln*'h200, read_data);
            if(read_data == 'h0)
                `uvm_info(get_full_name(),$sformatf("PHY%0d Bert errors on channel%0d expect 0 error",pn,ln),UVM_LOW)
            else
                `uvm_error(get_full_name(),$sformatf("PHY%0d Bert errors on channel%0d expect 0 error,read data is 0x%0x",pn,ln,read_data))
        end
        `uvm_info(get_full_name(),$sformatf("PHY%0d Introducing error",pn),UVM_LOW)
        jtag_cr_write(pn, 'h5071, 'h11);
        jtag_cr_write(pn, 'h5071, 'h1);
        `uvm_info(get_full_name(),$sformatf("PHY%0d Waiting for         10 microseconds for expected events to take place",pn),UVM_LOW)
        #10us;
        for(int ln=0; ln<4; ln++) begin
            jtag_cr_read(pn, 'h1094+ln*'h200, read_data);
            if(read_data == 'h1)
                `uvm_info(get_full_name(),$sformatf("PHY%0d Bert errors on channel%0d expect 1 error",pn,ln),UVM_LOW)
            else
                `uvm_error(get_full_name(),$sformatf("PHY%0d Bert errors on channel%0d expect 1 error,read data is 0x%0x",pn,ln,read_data))
        end
        `uvm_info(get_full_name(),$sformatf("PHY%0d disable RX LBERT matcher",pn),UVM_LOW)
        jtag_cr_write(pn, 'h5093, 'h0);
        `uvm_info(get_full_name(),$sformatf("PHY%0d disable RX datapath",pn),UVM_LOW)
        jtag_cr_write(pn, 'h6020, 'h80aa);
        `uvm_info(get_full_name(),$sformatf("PHY%0d disable TX LBERT in LFSR31",pn),UVM_LOW)
        jtag_cr_write(pn, 'h5071, 'h0);
        `uvm_info(get_full_name(),$sformatf("PHY%0d End of ATE test sequence           6",pn),UVM_LOW)
        //`uvm_info(get_full_name(),$sformatf("PHY%0d Waiting for         10 microseconds for expected events to take place",pn),UVM_LOW)
        //#10us;
    endtask

    task load_cfg(bit[1:0] pn,string load_file);
        integer fh,status;
        string  line;
        bit [31:0] addr,data;
        bit        rsp;

        fh = $fopen(load_file, "r");
        if(!fh) begin
            `uvm_info(get_full_name(),$sformatf("Failed file from path %s", load_file),UVM_LOW);
        end
        while (!$feof(fh)) begin
            // Get the next line
            if(!$fgets(line, fh)) continue;
            `uvm_info(get_full_name(),$sformatf("%s",line),UVM_LOW)
            if (line.substr(0, 9) == "CREG WRITE") begin
                status = $sscanf(line, "CREG WRITE : ADDR=%h, DATA=%h", addr, data);
                if(status == 2) begin
                     jtag_cr_write(pn, addr, data);
                     `uvm_info(get_full_name(),$sformatf("JTAG WRITE : phy%0d addr 0x%0x, data 0x%0x",pn,addr,data),UVM_LOW)
                end
                else
                  `uvm_warning(get_full_name(),$sformatf("%s read failed", line));
            end
        end
        $fclose(fh);
    endtask

    task wait_signal(string hier="", bit[31:0] val);
        bit[31:0] data;
        fork
            while(1) begin
                uvm_hdl_read(hier,data);
                if(data == val) break;
                #1ns;
            end
            begin
                #1ms;
                `uvm_error(get_full_name(),{hier,"wait timeout"})

            end
        join_any
        disable fork;

    endtask

    task jtag_tdr_driver(int jtag_sel);
        int_vif.SPI_0_DO_0<=1'b0;
        #1000ns;
        init_tap;           /*reset all tdr value*/
        select_ijtag_reg;
        enable_pcie;
        enable_pcie_sri_sib;
        enable_pcie_tm_tdr_sib;
        //enable_phy1;  /*enable_phy0 enable_phy1 enable_phy2 enable_phy3*/
        enable_phy_sel(jtag_sel);
        enable_pcie;
        enable_pcie_sri_sib;
        enable_pcie_phy_tdr_sib;
        select_phy_original(1'b0, 1'b0);  /*config phy enter jtag cfg mode*/
        @(posedge int_vif.SPI_0_DO_0);
        #1000ns;
        /* pcie_phy_reg_access */

        //$finish;
    endtask

    task enable_phy_sel(int jtag_sel);
        case(jtag_sel)
            0: enable_buf_die_top;
            1: enable_phy0;
            2: enable_phy1;
            3: enable_phy2;
            4: enable_phy3;
            default: `uvm_fatal("dbg_info", $sformatf("bad jtag_sel > 4!"))
        endcase
    endtask

    task test_mode_enable;
        sys_cfg_vif.test_mode<=1'b1;
        pcie_jtag_if.tdi<=1'b1;
    endtask

    task init_tap;
        test_mode_enable;
        //reset state
        int_vif.SPI_0_DI_0<=1'b0 ;
        @(negedge int_vif.SPI_0_DO_0);
        //run test idle
        int_vif.SPI_0_DI_0<=1'b1 ;
        int_vif.AON_JTAG_TMS_0<=1'b0 ;
        @(negedge int_vif.SPI_0_DO_0);
    endtask

    task write_tap_ir_reg;
        input [3:0] ir_data;
        integer i;
        //enter ir_scan state
        int_vif.AON_JTAG_TMS_0<=1'b1 ;
        @(negedge  int_vif.SPI_0_DO_0);
        int_vif.AON_JTAG_TMS_0<=1'b1 ;
        @(negedge  int_vif.SPI_0_DO_0);
        //enter shif ir state
        int_vif.AON_JTAG_TMS_0<=1'b0 ;
        @(negedge  int_vif.SPI_0_DO_0);
        int_vif.AON_JTAG_TMS_0<=1'b0 ;
        @(negedge  int_vif.SPI_0_DO_0);
        //shift ir data
        for(i=0;i<=3;i++) begin
            int_vif.AON_JTAG_TDI_0<=ir_data[i];
            if (i==3) begin
                int_vif.AON_JTAG_TMS_0<=1'b1 ;
            end
            @(negedge  int_vif.SPI_0_DO_0);
        end
        int_vif.AON_JTAG_TMS_0<=1'b1 ;
        @(negedge  int_vif.SPI_0_DO_0);
        //run test idle
        int_vif.AON_JTAG_TMS_0<=1'b0 ;
        @(negedge  int_vif.SPI_0_DO_0);
    endtask

    task write_tap_dr_reg;
        input [69:0]  dr_data; 
        integer i;
        //enter id_scan state
        int_vif.AON_JTAG_TMS_0<=1'b1 ;
        @(negedge  int_vif.SPI_0_DO_0);
        //enter shif dr state
        int_vif.AON_JTAG_TMS_0<=1'b0 ;
        @(negedge  int_vif.SPI_0_DO_0);
        int_vif.AON_JTAG_TMS_0<=1'b0 ;
        @(negedge  int_vif.SPI_0_DO_0);
        //shift dr data
        for(i=0;i<=69;i++) begin
            int_vif.AON_JTAG_TDI_0<=dr_data[i];
            if (i==69) begin
                int_vif.AON_JTAG_TMS_0<=1'b1 ;
            end
            @(negedge  int_vif.SPI_0_DO_0);
        end
        int_vif.AON_JTAG_TMS_0<=1'b1 ;
        @(negedge  int_vif.SPI_0_DO_0);
        //run test idle
        int_vif.AON_JTAG_TMS_0<=1'b0 ;
        @(negedge  int_vif.SPI_0_DO_0);
    endtask

    task select_ijtag_reg;
        write_tap_ir_reg(4'b0000);
    endtask

    task enable_pcie;
        write_tap_dr_reg({3'b001,67'b0000000000_0000000000_0000000000_0000000000_0000000000_0000000000_0000000});
        write_tap_dr_reg({7'b0010001,63'b0000000000_0000000000_0000000000_0000000000_0000000000_0000000000_000});
        write_tap_dr_reg({14'b0000000101_0001,56'b0000000000_0000000000_0000000000_0000000000_0000000000_000000});
        write_tap_dr_reg({15'b0000000110_10001,55'b0000000000_0000000000_0000000000_0000000000_0000000000_00000});
        write_tap_dr_reg({39'b0000000000_0000000000_0010000000_011010001,31'b0000000000_0000000000_0000000000_0});
        write_tap_dr_reg({43'b0000000000_0000000000_0000101000_0000011010_001,27'b0000000000_0000000000_0000000});
    endtask

    task enable_pcie_sri_sib;
        write_tap_dr_reg({45'b0000000000_0000000000_0000011010_0000000110_10001,25'b0000000000_0000000000_00000});
    endtask

    task enable_pcie_tm_tdr_sib;
        write_tap_dr_reg({61'b0000000000_0000000000_0000000000_1000000000_1110100000_0001101000_1,9'b000000000});
    endtask

    task enable_pcie_phy_tdr_sib;
        write_tap_dr_reg({61'b0000000000_0000000000_0000000001_0000000000_1110100000_0001101000_1,9'b000000000});
    endtask

    task enable_pcie_tm_tdr;
        input TM1;
        input TM2;
        input TM3;
        $display("TM1=%b,TM2=%b,TM3=%b",TM1,TM2,TM3);
        write_tap_dr_reg({33'b0000000000_0000000000_0000000000,TM1,TM2,TM3,37'b0000000000_0000000000_0000000000_0000000});
    endtask

    task select_phy_original;
        input sel0;
        input sel1;
        write_tap_dr_reg({29'b0000000000_0000000000_000000000,sel0,sel1,sel0,sel1,sel0,sel1,sel0,sel1,32'b0_0000000000_000000000_0000000000_00,1'b0});
    endtask

    task enable_buf_die_top;
        enable_pcie_tm_tdr(1'b0,1'b0,1'b0);
    endtask

    task enable_phy0;
        enable_pcie_tm_tdr(1'b1,1'b0,1'b0);
    endtask

    task enable_phy1;
        enable_pcie_tm_tdr(1'b0,1'b1,1'b0);
    endtask

    task enable_phy2;
        enable_pcie_tm_tdr(1'b1,1'b1,1'b0);
    endtask

    task enable_phy3;
        enable_pcie_tm_tdr(1'b0,1'b0,1'b1);
    endtask


endclass

`endif
