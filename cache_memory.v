`timescale 1ns / 1ps

module cache #(parameter cache_sz = 64, block_sz = 128, addr_sz = 10, data_sz = 32) (
    input isWrite,
    input [addr_sz-1:0] addr,
    input [data_sz-1:0] Wdata,
    input [data_sz-1:0] mem_Rdata,
    input Done,
    output reg [data_sz-1:0] Rdata,
    output reg hit,
    output reg mem_Write,
    output reg [addr_sz-1:0] mem_addr,
    output reg [data_sz-1:0] mem_Wdata
);
    reg [1:0] status, next;
    reg [7:0] cache_reg [0:3] [15:0];
    reg [5:0] mark [0:3]; // valid + dirty + tag
    reg [3:0] tag;
    reg [1:0] index;
    reg [3:0] word;
    reg [1:0] cnt;

    // initial
    integer i, j;
    initial begin 
        {next, cnt, mem_Write} = 0;
        hit = 1;
        for (i=0;i<4;i=i+1) begin 
            for (j=0;j<16;j=j+1) begin cache_reg[i][j] = 0; end
            mark[i] = 0; 
        end
    end
    
    // fsm beginning
    always @(*) begin hit = (mark[index][5] & (tag == mark[index][3:0])); end
    always @(isWrite, addr, Wdata) begin 
        next = 2'b01; 
        tag = addr[9:6];
        index = addr[5:4];
        word = addr[5:2];
    end

    // miss fsm
    always @(posedge Done) begin
        case (status)
            2'b10: begin
                mark[index][4] = 0;
                next = (cnt == 2'b11)? 2'b11:2'b10;
                cnt = (cnt == 2'b11)? 0 : cnt + 1;
            end
            2'b11: begin
                mark[index][3:0] = tag;
                mark[index][5] = 1;
                {cache_reg[index][{cnt, 2'h3}], 
                    cache_reg[index][{cnt, 2'h2}], 
                    cache_reg[index][{cnt, 2'h1}], 
                    cache_reg[index][{cnt, 2'h0}]
                } = mem_Rdata; 
                next = (cnt == 2'b11)? 2'b01:2'b11; 
                cnt = (cnt == 2'b11)? 0 : cnt + 1;
            end
        endcase
    end

    // hit
    always @(status == 2'b01) begin
        if (hit) begin
            next = 2'b0;
            if (isWrite) begin
                if (addr[1:0] == 0) begin 
                    {cache_reg[index][{addr[3:2], 2'h3}], cache_reg[index][{addr[3:2], 2'h2}], 
                        cache_reg[index][{addr[3:2], 2'h1}], cache_reg[index][{addr[3:2], 2'h0}]} = Wdata;
                end
                else cache_reg[index][addr[3:0]] = Wdata;
                mark[index][4] = 1;
            end
            else begin 
                if (addr[1:0] == 0) begin 
                    Rdata = {cache_reg[index][{addr[3:2], 2'h3}], 
                            cache_reg[index][{addr[3:2], 2'h2}], 
                            cache_reg[index][{addr[3:2], 2'h1}], 
                            cache_reg[index][{addr[3:2], 2'h0}]}; 
                end
                else Rdata = {{25{cache_reg[index][addr[3:0]][7]}}, cache_reg[index][addr[3:0]][6:0]};
            end
        end
        else begin next = (mark[index][5:4] == 2'b11)? 2'b10:2'b11; end
    end


    // state transition
    always @(*) begin 
        status = next; 
        if (next == 2'b10) begin
           mem_Wdata <= {cache_reg[index][{cnt, 2'h3}], 
                            cache_reg[index][{cnt, 2'h2}], 
                            cache_reg[index][{cnt, 2'h1}], 
                            cache_reg[index][{cnt, 2'h0}]}; 
           mem_Write <= 1;
           mem_addr <= {mark[index][3:0], index, cnt, 2'b00};
        end
        else begin
            mem_Wdata <= 0;
            mem_Write <= 0;
            mem_addr <= {tag, index, cnt, 2'b00};
        end
    end
endmodule

module main_mem #(parameter addr_sz = 10, mem_sz = 1024, data_sz = 32) (
    input [addr_sz-1:0] addr,
    input isWrite,
    input [data_sz-1:0] Wdata,
    output reg [data_sz-1:0] Rdata,
    output reg Done
);
    reg [7:0] mem_reg [0:mem_sz-1];
    integer i;
    initial begin 
        Done = 1;
        Rdata = 0;
        for(i=0;i<mem_sz;i=i+1) mem_reg[i] = 0;
        $readmemh("./bin.txt", mem_reg);
    end
    always @(*) begin
        Done = 0;
        if (isWrite) begin {mem_reg[addr+3], mem_reg[addr+2], mem_reg[addr+1], mem_reg[addr]} = Wdata; end
        else begin Rdata = {mem_reg[addr+3], mem_reg[addr+2], mem_reg[addr+1], mem_reg[addr]}; end
        // #0.5 Done <= 1;
        Done <= 1;
    end
endmodule

module CPU (
    input  hit_miss,
    input  clock,
    output read_write,
    output [9:0] address,
    output [31:0] write_data
);
    parameter  request_total = 10; // change this number to how many requests you want in your testbench
    reg [4:0]  request_num;
    reg        read_write_test[request_total-1:0];
    reg [9:0]  address_test[request_total-1:0];
    reg [31:0] write_data_test[request_total-1:0]; 
    initial begin
        #1 request_num = 0;
        read_write_test[0] = 0; address_test[0] = 10'h0; write_data_test[0] = 0; // miss read
        read_write_test[1] = 1; address_test[1] = 10'h10; write_data_test[1] = 32'hfac; // miss write
        read_write_test[2] = 0; address_test[2] = 10'h14; write_data_test[2] = 0; // hit read
        read_write_test[3] = 0; address_test[3] = 10'h46; write_data_test[3] = 0; // miss read
        read_write_test[4] = 0; address_test[4] = 10'h10; write_data_test[4] = 0; // hit read 
        read_write_test[5] = 1; address_test[5] = 10'h42; write_data_test[5] = 32'h3f; // hit write
        read_write_test[6] = 1; address_test[6] = 10'h0; write_data_test[6] = 32'hffffffff; // miss write
        read_write_test[7] = 1; address_test[7] = 10'h8; write_data_test[7] = 32'hffffffff; // miss write
        read_write_test[8] = 0; address_test[8] = 10'h40; write_data_test[8] = 0; // miss read
        read_write_test[9] = 0; address_test[9] = 10'h1; write_data_test[9] = 0; // miss read
    end
    always @(posedge clock) begin
        if (hit_miss == 1) request_num = request_num + 1;
        else request_num = request_num;
    end
    assign address      = address_test[request_num];
    assign read_write   = read_write_test[request_num];
    assign write_data   = write_data_test[request_num]; 
endmodule

module lab6 (
    input isWrite,
    input [9:0] addr,
    input [31:0] Wdata,
    output [31:0] Rdata,
    output hit 
);
    wire mem_Write;
    wire [9:0] mem_addr;
    wire [31:0] mem_Wdata, mem_Rdata;
    wire Done;

    cache utt1(
        isWrite,
        addr,
        Wdata,
        mem_Rdata,
        Done,
        Rdata,
        hit,
        mem_Write,
        mem_addr,
        mem_Wdata
    );

    main_mem utt2(
        mem_addr,
        mem_Write,
        mem_Wdata,
        mem_Rdata,
        Done
    );
endmodule