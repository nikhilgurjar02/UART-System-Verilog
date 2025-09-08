class transaction;
  
  typedef enum bit  {write = 1'b0 , read = 1'b1} oper_type;
  randc oper_type oper;
  bit rx;               //rx from 1-->0 means we will receive data from next uart_clk
  rand bit [7:0] dintx; //Data to send through tx
  bit newd;             //newd = high --> means we want to send data
  bit tx;               //dintx will be output serially (starting from LSB)
  bit [7:0] doutrx;     //once all data is received from rx collect through Shift Register and send in output
  bit donetx;           //all 7 bits of dintx are sent
  bit donerx;           //7 bits of data received from rx in doutrx
  
  function transaction copy();
    copy = new();
    copy.rx = this.rx;
    copy.dintx = this.dintx;
    copy.newd = this.newd;
    copy.tx = this.tx;
    copy.doutrx = this.doutrx;
    copy.donetx = this.donetx;
    copy.donerx = this.donerx;
    copy.oper = this.oper;
  endfunction
  
endclass
 
 
class generator;
  
 transaction tr; 
  mailbox #(transaction) mbx;
  event done;
  event drvnext;
  event sconext;
  int count = 0;        //How many testcases we want to test
  
  function new(mailbox #(transaction) mbx);
    this.mbx = mbx;
    tr = new();
  endfunction
  
  task run();
  
    repeat(count) begin
      assert(tr.randomize) else $error("[GEN] :Randomization Failed");
      mbx.put(tr.copy);
      $display("[GEN]: Oper : %0s Din : %0d",tr.oper.name(), tr.dintx);
      @(drvnext);
      @(sconext);
    end
    
    -> done;
  endtask
  
  
endclass
 
/////////////////////////////////////////////////////////
class driver;
 
  virtual uart_if vif; 
  transaction tr;
  mailbox #(transaction) mbx;
  mailbox #(bit [7:0]) mbxds; 
  event drvnext;
  
  bit [7:0] datarx;  ///data rcvd during read to be sent for reference
  
  function new(mailbox #(bit [7:0]) mbxds, mailbox #(transaction) mbx);
    this.mbx = mbx;
    this.mbxds = mbxds;
   endfunction
  
  task reset();
    vif.rst <= 1'b1;
    vif.dintx <= 0;
    vif.newd <= 0;
    vif.rx <= 1'b1;
 
    repeat(5) @(posedge vif.uclktx);
    vif.rst <= 1'b0;
    @(posedge vif.uclktx);
    $display("[DRV] : RESET DONE");
    $display("----------------------------------------");
  endtask
  
  task run();
  
    forever begin
      mbx.get(tr);
      
      if(tr.oper == 1'b0)  ////data transmission/write operation
          begin
          //           
            @(posedge vif.uclktx);
            vif.rst <= 1'b0;
            vif.newd <= 1'b1;   //we have newdata which we want to send/transmit
            vif.rx <= 1'b1;     //Keep rx high because we are not reading data     
            vif.dintx = tr.dintx;   //data to be sent is randomized and received from generator through mailbox
            @(posedge vif.uclktx);
            vif.newd <= 1'b0;
         
            mbxds.put(tr.dintx);
            $display("[DRV]: Data Sent : %0d", tr.dintx);
             wait(vif.donetx == 1'b1);  //wait for done signal from dut to proceed
             ->drvnext;  
          end
      
      else if(tr.oper == 1'b1)  //Data receive/read operation
               begin
                 @(posedge vif.uclkrx);
                  vif.rst <= 1'b0;
                  vif.rx <= 1'b0;       //rx will be low for one clock to make device ready for receiving
                  vif.newd <= 1'b0;     //newd = 0 during read
                  @(posedge vif.uclkrx);
                  
                 for(int i=0; i<=7; i++) 
                 begin   
                      @(posedge vif.uclkrx);                
                      vif.rx <= $urandom;   //generate random 7 bits serially for receiver
                      datarx[i] = vif.rx;    //data received should be sent for reference through mailbox                                  
                 end 
                 
                mbxds.put(datarx);
                
                $display("[DRV]: Data RCVD : %0d", datarx); 
                wait(vif.donerx == 1'b1);   //wait for done signal from dut to proceed
                 vif.rx <= 1'b1;            //make RX again High for letting device know for stop receiving
				->drvnext;        
             end             
      end
  endtask
endclass
 

 
class monitor;
 
  transaction tr;
  mailbox #(bit [7:0]) mbx;
  virtual uart_if vif;
  
  bit [7:0] srx; //data sent serially from transmitter is sent to scoreboard
  bit [7:0] rrx; //data received from receiver is sent to scoreboard

  function new(mailbox #(bit [7:0]) mbx);
    this.mbx = mbx;
    endfunction
  
  task run(); 
    forever begin
       @(posedge vif.uclktx);
            //newd=1 --> we have new data to send & rx=1 -->not receiving data
      if ( (vif.newd== 1'b1) && (vif.rx == 1'b1) )   //operation = Transmit data or write
                begin   
                  @(posedge vif.uclktx); ////start collecting tx data from next clock tick   
              for(int i = 0; i<= 7; i++) 
              begin     
                    @(posedge vif.uclktx);
                    srx[i] = vif.tx;         
              end
                  $display("[MON] : DATA SEND on UART TX %0d", srx);    
                @(posedge vif.uclktx); //
                mbx.put(srx);     
               end
      
      else if ((vif.rx == 1'b0) && (vif.newd == 1'b0) ) //rx = 0 --> receiving data & newd = 0 --> not transmitting 
        begin
          wait(vif.donerx == 1);
           rrx = vif.doutrx;     
           $display("[MON] : DATA RCVD RX %0d", rrx);
           @(posedge vif.uclktx); 
           mbx.put(rrx);
      end
  end  
endtask
  
 
endclass
 
////////////////////////////////////////////////////////
class scoreboard;
  mailbox #(bit [7:0]) mbxds, mbxms;
  
  bit [7:0] ds;
  bit [7:0] ms;
  
   event sconext;
  
  function new(mailbox #(bit [7:0]) mbxds, mailbox #(bit [7:0]) mbxms);
    this.mbxds = mbxds;
    this.mbxms = mbxms;
  endfunction
  
  task run();
    forever begin    
      mbxds.get(ds);
      mbxms.get(ms);
      
      $display("[SCO] : DRV : %0d MON : %0d", ds, ms);
      if(ds == ms)
        $display("DATA MATCHED");
      else
        $display("DATA MISMATCHED");
      $display("----------------------------------------");
      
     ->sconext; 
    end
  endtask  
endclass
 
///////////////////////////////
 
class environment;
 
    generator gen;
    driver drv;
    monitor mon;
    scoreboard sco; 
    virtual uart_if vif;
    
    event nextgd; ///gen -> drv
    event nextgs;  /// gen -> sco
  
  mailbox #(transaction) mbxgd; ///gen - drv
  mailbox #(bit [7:0]) mbxds; /// drv - sco 
  mailbox #(bit [7:0]) mbxms;  /// mon - sco
  
  function new(virtual uart_if vif);    
    mbxgd = new();
    mbxms = new();
    mbxds = new();
    gen = new(mbxgd);
    drv = new(mbxds,mbxgd);
    mon = new(mbxms);
    sco = new(mbxds, mbxms);
    
    this.vif = vif;
    drv.vif = this.vif;
    mon.vif = this.vif;
    
    gen.sconext = nextgs;
    sco.sconext = nextgs;
    gen.drvnext = nextgd;
    drv.drvnext = nextgd;
  endfunction
  
  task pre_test();
    drv.reset();
  endtask
  
  task test();
  fork
    gen.run();
    drv.run();
    mon.run();
    sco.run();
  join_any
  endtask
  
  task post_test();
    wait(gen.done.triggered);  
    $finish();
  endtask
  
  task run();
    pre_test();
    test();
    post_test();
  endtask
  
endclass
 
///////////////////////////////////////////
module tb;
  environment env;  
  uart_if vif();
  uart_top #(1000000, 9600) dut (vif.clk,vif.rst,vif.rx,vif.dintx,vif.newd,vif.tx,vif.doutrx,vif.donetx, vif.donerx);
  
    initial begin
      vif.clk <= 0;
    end
    
    always #10 vif.clk <= ~vif.clk;
    
    initial begin
      env = new(vif);
      env.gen.count = 5;
      env.run();
    end
      
    
    initial begin
      $dumpfile("dump.vcd");
      $dumpvars;
    end
   
  assign vif.uclktx = dut.utx.uclk;
  assign vif.uclkrx = dut.rtx.uclk;
    
  endmodule