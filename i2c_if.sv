interface i2c_if #(
    int ADDR_WIDTH = 32,
    int DATA_WIDTH = 16
)(
    input logic SCL,
    inout logic SDA
);
parameter I2C_DATA_WIDTH = 8;    
parameter I2C_ADDR_WIDTH = 7;  // cus 1 bit is of R/W 
typedef enum bit {
    WRITE = 0,    // R/W bit = 0 means master is WRITING
    READ  = 1     // R/W bit = 1 means master is READING
} i2c_op_t;
 
	



// ****************************************************************************              
// Do i need the reset Bus part ???? 
// ****************************************************************************   
// Task #1 (Waits for and capturess transfer start)
task wait_for_i2c_transfer ( output i2c_op_t op, output bit[I2C_DATA_WIDTH-1:0] write_data []); // Output: R/W (op) Output: Shows the written dtat (write_data)
bit [7:0] addr_byte;
bit rw_bit;
bit stop_detected;
bit [7:0] current_byte;                      
bit [I2C_DATA_WIDTH-1:0] data_queue[$];      

	// Step 1 : Wait for START condition
	@(negedge SDA iff (SCL == 1)); // Only trigger on negedge SDA IF SCL == 1 at that moment

	// Step 2: Store address and R/W
	for (int i = 7; i >= 0; i--) begin // Store here address
	    @(posedge SCL);
	    addr_byte[i] = SDA;
	end
	
	// Step 3: Check R/W
	rw_bit = addr_byte[0]; // Store R/W bit (last bit [0])
	if (rw_bit == 1'b0)
		op = WRITE;
	else
		op = READ;
	

	// Step 4: Send ACK
	@(negedge SCL);
	SDA = 1'b0;
	@(posedge SCL);
	@(negedge SCL);
	SDA = 1'bz;

	if (op == READ) return; // if Read then end TASK#1 
	
	// Step 5 : Collect Data Bytes (Write Only) + Wait for Stop Condition 
	stop_detected = 0;

	while (!stop_detected) begin
		fork 
			begin : next_byte

				//@(posedge SCL iff (SDA == 0)); // Received ACK : SDA low while clock high (can i skip this as its write 														     Condition ??)
				for (int i = 7; i >= 0; i--) begin
					@(posedge SCL);
				        current_byte[i] = SDA;
				end
				data_queue.push_back(current_byte); // push to a queue
				
				// Now send ACK for this byte
				@(negedge SCL);    // wait for SCL LOW
				SDA = 1'b0;        // pull SDA LOW = ACK
				@(posedge SCL);    // master clocks in ACK
				@(negedge SCL);    // wait for SCL LOW again
				SDA = 1'bz;        // release SDA
			end
			begin : stop_condition
				@(posedge SDA iff (SCL == 1)); //Stop Condtion received 
				stop_detected = 1;
			end
		join_any	// First one done goes forwards
		disable fork;   // Kill the not used fork
	end

	// Step 6: Convert queue to output    // Store into a queue
	write_data = new[data_queue.size()];
	foreach(data_queue[i])
		write_data[i] = data_queue[i];

endtask
// ****************************************************************************    
// Task #2 (Provides data for read operation)
task provide_read_data (input bit [I2C_DATA_WIDTH-1:0] read_data [],output bit transfer_complete); // Input : Data to sent for Read (read_data) Output: tells is 														transfer is complete (transfer_complete)
	
	transfer_complete = 0;

	// Step 1: Loop through each byte
	foreach (read_data[byte_idx]) begin
        
	// Step 2: Send 8 bits MSB first
	for (int i = 7; i >= 0; i--) begin
		@(negedge SCL);                    // wait SCL LOW
		SDA = read_data[byte_idx][i];      // drive bit safely
	end
	
	 // Step 3: Release SDA for 9th clock
        @(negedge SCL);
        SDA = 1'bz;                            // let go of SDA

// Step 4: Check ACK or NACK
        @(posedge SCL);                        // clock rises
        
        if (SDA == 1'b1) begin
            // NACK - master done reading
            transfer_complete = 1;
            @(negedge SCL);
            SDA = 1'bz;
            return;
        end
        // ACK - continue foreach to next byte

    end

// Step 6: All bytes sent successfully
    transfer_complete = 1;	
	
endtask
// ****************************************************************************    
//Task #3 (Returns data observed)
task monitor (output bit [I2C_ADDR_WIDTH-1:0] addr, output i2c_op_t op, output bit [I2C_DATA_WIDTH-1:0] data []);// Output : Addr , op , data
    bit [7:0] addr_byte;
    bit [7:0] current_byte;
    bit       stop_detected;
    bit [I2C_DATA_WIDTH-1:0] data_queue[$];


// Step 1 : Wait for START condition
	@(negedge SDA iff (SCL == 1)); // Only trigger on negedge SDA IF SCL == 1 at that moment

// Step 2: Store address and R/W
	for (int i = 7; i >= 0; i--) begin // Store here address
	    @(posedge SCL);
	    addr_byte[i] = SDA;
	end
	// Extract address and R/W bit
	addr = addr_byte[7:1];             // top 7 bits = address
	op   = i2c_op_t'(addr_byte[0]);   // bit 0 = R/W

//Step 3 : Skip ACK bit
@(posedge SCL);    // 9th clock HIGH
@(negedge SCL);    // 9th clock LOW

// Step 4: Collect all data bytes
	stop_detected = 0;

	while (!stop_detected) begin
		fork
		        // Thread 1: Watch for incoming byte
		    	begin : byte_thread
				// Read 8 bits
					for (int i = 7; i >= 0; i--) begin
					    @(posedge SCL);
					    current_byte[i] = SDA;
					end

				// Save byte to queue
				data_queue.push_back(current_byte);

				// Skip ACK/NACK clock
				@(posedge SCL);    // 9th clock HIGH
				@(negedge SCL);    // 9th clock LOW
		    	end

		        // Thread 2: Watch for STOP condition
		        begin : stop_thread
		        	@(posedge SDA iff (SCL == 1));
		        	stop_detected = 1;
		        end

                join_any
                disable fork;
        end

   
// Step 5: Convert queue to output array
    
	data = new[data_queue.size()];
	foreach(data_queue[i])
		data[i] = data_queue[i];


endtask
// ****************************************************************************    
endinterface
