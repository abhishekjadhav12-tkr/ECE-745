`timescale 1ns / 10ps
module top();
// changes

parameter int WB_ADDR_WIDTH = 2;
parameter int WB_DATA_WIDTH = 8;
parameter int NUM_I2C_BUSSES = 8;

logic  clk;
logic  rst;
wire cyc;
wire stb;
wire we;
tri1 ack;
wire [WB_ADDR_WIDTH-1:0] adr;
wire [WB_DATA_WIDTH-1:0] dat_wr_o;
wire [WB_DATA_WIDTH-1:0] dat_rd_i;
wire irq;
tri1  [NUM_I2C_BUSSES-1:0] scl;
tri1  [NUM_I2C_BUSSES-1:0] sda;

//logic [WB_DATA_WIDTH-1:0] data; // Added by me 

// ****************************************************************************
// Clock generator Changed
initial begin : clk_gen
  clk = 0;
end

always begin
 #5 clk = ~clk;  // 10 ns period
end


// ****************************************************************************
// Reset generator
initial begin
  rst = 1;
  #113;
  rst = 0;
end


// ****************************************************************************
// Monitor Wishbone bus and display transfers in the transcript
bit [31:0] monitor_addr;
bit [31:0] monitor_data;
bit        monitor_we;

initial begin : wb_monitoring
    wait(rst==0);
     
       forever begin
            #100;
            wb_bus.master_monitor(monitor_addr, monitor_data, monitor_we);
            $display("Time : [%0t] %s Addr=0x%0h Data=0x%0h", $time, monitor_we ? "CMD : WRITE   " : "CMD : READ   ",
                    monitor_addr, monitor_data);
        end
    
end


// ****************************************************************************
// Define the flow of the simulation

// Register addresses Page 18
parameter CSR_ADDR  = 2'h0;  
parameter DPR_ADDR  = 2'h1;
parameter CMDR_ADDR = 2'h2; 
parameter FSMR_ADDR = 2'h3;



// Define the task (this is fine at module level)

task example_1();
  	wb_bus.master_write(CSR_ADDR, 8'b11000000); // Hex : C0
	$display("Example 1 DONE");
endtask

task example_3();
    bit [7:0] status;
    
    // Step 1: ID of desired I2C bus
    wb_bus.master_write(DPR_ADDR, 8'h05);
    $display("Step 1 Done");
    
    // Step 2: Set Bus command
    wb_bus.master_write(CMDR_ADDR, 8'b110);
    $display("Step 2 Done");
    
    // Step 3: Wait for interrupt (poll until DON bit is set)------------> interrupt no.1
    do begin
        wb_bus.master_read(CMDR_ADDR, status);
//$display("Time=%0t: Waiting Step 3, Status = 0x%0h (binary: %0b), IRQ = %0b", $time, status, status, irq); //
//$display(" Inside interrupt no.1 ");
    end while (status[7:4] == 4'b0000 || irq == 1'h1);  // Wait until DON bit = 1
    $display("Step 3 Done");
//$display("Time=%0t: Before Step 4, Status = 0x%0h (binary: %0b), IRQ = %0b", $time, status, status, irq);
    // Step 4: Start command
    wb_bus.master_write(CMDR_ADDR, 8'b100);
    $display("Step 4 Done");
	wb_bus.master_read(CMDR_ADDR, status);
//$display("Time=%0t: After Step 4, Status = 0x%0h (binary: %0b), IRQ = %0b", $time, status, status, irq);
   
    // Step 5: Wait for interrupt (poll until DON bit is set)------------> interrupt no.2
    do begin
	@(posedge clk);
$display(" Inside interrupt no.2 ");
//$display("Time=%0t: Before Step 5, Status = 0x%0h (binary: %0b), IRQ = %0b", $time, status, status, irq);
        wb_bus.master_read(CMDR_ADDR, status);
	
//$display("Time=%0t: Waiting Step 5, Status = 0x%0h (binary: %0b)", $time, status, status);
//$display("Time=%0t: Waiting Step 5, Status = 0x%0h (binary: %0b), IRQ = %0b", $time, status, status, irq);
    end while (status[7:4] == 4'b0000 || irq == 1'h1);  // Wait until DON bit = 1 //
 
    // Step 6: Write byte 0x44 to the DPR
    wb_bus.master_write(DPR_ADDR, 8'h44);
    
    // Step 7: Write command
    wb_bus.master_write(CMDR_ADDR, 8'b001);
    $display("Step 5,6 and 7 Done");
    
    // Step 8: Wait for interrupt ------------> interrupt no.3
    do begin
$display(" Inside interrupt no.3 ");
       // wb_bus.master_read(CMDR_ADDR, status);
	@(posedge clk);
	wb_bus.master_read(CMDR_ADDR, status);
    end while (status[7:4] == 4'b0000 || irq == 1'h1);
    
    // Step 9: Write byte 0x78
    wb_bus.master_write(DPR_ADDR, 8'h78);
    
    // Step 10: Write command
    wb_bus.master_write(CMDR_ADDR, 8'b001);
    $display("Step 8,9 and 10 Done");
//$display("Time=%0t: Waiting Step 11, Status = 0x%0h (binary: %0b)", $time, status, status);    
    // Step 11: Wait for interrupt ------------> interrupt no.4
    do begin
$display(" Inside interrupt no.4 ");
//$display("Time=%0t: Inside Step 11, Status = 0x%0h (binary: %0b)", $time, status, status);
	@(posedge clk);
	wb_bus.master_read(CMDR_ADDR, status);
    end while (status[7:4] == 4'b0000 || irq == 1'h1);
    
    // Step 12: Stop command
    wb_bus.master_write(CMDR_ADDR, 8'b101);
    $display("Step 11 and 12 Done");
    
    // Step 13: Wait for interrupt ------------> interrupt no.5.
    do begin
$display(" Inside interrupt no.5 ");     
	@(posedge clk);
	wb_bus.master_read(CMDR_ADDR, status);
    end while (status[7:4] == 4'b0000 || irq == 1'h1);
    
    $display("Example 3 Done");
endtask

// Call the task INSIDE an initial block
initial begin
  wait(rst == 0);       // Wait for reset
  @(posedge clk);       // Wait one clock
  example_1();          // NOW call the task
  example_3();
  #100;
  $finish;
end


// ****************************************************************************
// Instantiate the Wishbone master Bus Functional Model
wb_if       #(
      .ADDR_WIDTH(WB_ADDR_WIDTH),
      .DATA_WIDTH(WB_DATA_WIDTH)
      )
wb_bus (
  // System sigals
  .clk_i(clk),
  .rst_i(rst),
  // Master signals
  .cyc_o(cyc),
  .stb_o(stb),
  .ack_i(ack),
  .adr_o(adr),
  .we_o(we),
  // Slave signals
  .cyc_i(),
  .stb_i(),
  .ack_o(),
  .adr_i(),
  .we_i(),
  // Shred signals
  .dat_o(dat_wr_o),
  .dat_i(dat_rd_i)
  );

// ****************************************************************************
// Instantiate the DUT - I2C Multi-Bus Controller
\work.iicmb_m_wb(str) #(.g_bus_num(NUM_I2C_BUSSES)) DUT
  (
    // ------------------------------------
    // -- Wishbone signals:
    .clk_i(clk),         // in    std_logic;                            -- Clock
    .rst_i(rst),         // in    std_logic;                            -- Synchronous reset (active high)
    // -------------
    .cyc_i(cyc),         // in    std_logic;                            -- Valid bus cycle indication
    .stb_i(stb),         // in    std_logic;                            -- Slave selection
    .ack_o(ack),         //   out std_logic;                            -- Acknowledge output
    .adr_i(adr),         // in    std_logic_vector(1 downto 0);         -- Low bits of Wishbone address
    .we_i(we),           // in    std_logic;                            -- Write enable
    .dat_i(dat_wr_o),    // in    std_logic_vector(7 downto 0);         -- Data input
    .dat_o(dat_rd_i),    //   out std_logic_vector(7 downto 0);         -- Data output
    // ------------------------------------
    // ------------------------------------
    // -- Interrupt request:
    .irq(irq),           //   out std_logic;                            -- Interrupt request
    // ------------------------------------
    // ------------------------------------
    // -- I2C interfaces:
    .scl_i(scl),         // in    std_logic_vector(0 to g_bus_num - 1); -- I2C Clock inputs
    .sda_i(sda),         // in    std_logic_vector(0 to g_bus_num - 1); -- I2C Data inputs
    .scl_o(scl),         //   out std_logic_vector(0 to g_bus_num - 1); -- I2C Clock outputs
    .sda_o(sda)          //   out std_logic_vector(0 to g_bus_num - 1)  -- I2C Data outputs
    // ------------------------------------
  );


endmodule
