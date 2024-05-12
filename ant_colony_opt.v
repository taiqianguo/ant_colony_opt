`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 09.05.2024 15:19:08
// Design Name: 
// Module Name: controller
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module ant_colony_opt(
    input sys_clk_p,
    input sys_clk_n,
    input rst,
    //input get_solution, // when i stall all other process to scan ph map
    output [1:0]step// scanned out soluton 
    );
    
    
    wire clk;
    
    IBUFDS instan(
    .O(clk),
    .I(sys_clk_p),
    .IB(sys_clk_n)
    );
    
    parameter EV_FREQ=10000;// after 10000 clk freq , evaporate ph in exponential form * 3/4 
    parameter NUM_AGENTS = 8;
    parameter POS_ADDR=4;// 10<2^4
    parameter PH_DATA_SIZE=16;// 16 bit ph
    parameter MAP_SIZE=100;//square of map length  k^2
    parameter  MAP_LEN=10;
    
    parameter BACK_PR=3;// when solved and go back each back step add this ph
    
    //memory access signals
    wire map_douta;
    wire map_doutb;
    reg [POS_ADDR*2-1:0]map_addra=0;
    reg [POS_ADDR*2-1:0]map_addrb=0;
    
    wire [PH_DATA_SIZE-1:0]ph0;
    wire [PH_DATA_SIZE-1:0]ph1;
    reg [POS_ADDR*2-1:0]ph_addra=0;
    reg [POS_ADDR*2-1:0]ph_addrb=0;
    
    
    reg stall=0; // in stalling state no mem for agents are availble , process evaporate
    
    //connected to controller
    wire [NUM_AGENTS*POS_ADDR*2-1:0]positions;
    wire [NUM_AGENTS-1:0]solved_outs;
    wire [NUM_AGENTS*2-1:0]steps;
    wire [NUM_AGENTS-1:0] bus_avs;
    wire [NUM_AGENTS-1:0] bus_requests;
    
    //controller outputs for mem controll
    wire [POS_ADDR*2-1:0] pos_active;
    wire [1:0] step_active;
    wire solved_out;
    wire [1:0] grant_count;
    
    
    genvar i;
    generate
        for (i = 0; i < NUM_AGENTS; i=i+1) 
            begin
            // Declare unique wires for each agent instance
            wire [POS_ADDR*2-1:0] pos_now_i;    // Renamed to include index i
            wire [1:0] step_i;                 // Renamed to include index i
            wire solved_i;                     // Renamed to include index i
            wire bus_av_i;                     // Declare wire properly
            wire bus_request_i;                // Declare wire properly
    
            // Instantiate the agent with parameterized configuration
            //different agent with different , otherwise will explore the same path for all agents
            ant_agent#(.RSEED(128*i) ) agent (
                .clk(clk),
                .rst(rst),
                .solved_out(solved_i),
                .bus_av(bus_av_i),
                .bus_request(bus_request_i),
                .pos_now(pos_now_i),
                .step(step_i),
                .data0_map(map_douta),
                .data1_map(map_doutb),
                .ph0(ph0),
                .ph1(ph1)
            );
    
            // Connect agent outputs to system outputs or further processing
            assign positions[i*POS_ADDR*2 +: POS_ADDR*2] = pos_now_i;
            assign steps[i*2 +: 2] = step_i;
            assign solved_outs[i] = solved_i;
            assign bus_av_i= bus_avs[i];
            assign bus_requests[i] = bus_request_i;
            end
    endgenerate
        
    //intialize controller
    controller controller1(
    .clk(clk),
    .rst(rst),
    .stall(stall),
    .solved(solved_outs),
    .bus_request(bus_requests),
    .bus_av(bus_avs),
    .steps(steps),
    .poses_now(positions),
    .pos_active(pos_active),
    .step_active(step_active),
    .solved_out(solved_out),
    .grant_count(grant_count)
    
    );
    
    // stall and ph evaporate system , address transfer
    //inputs :   pos step solved grant_counter

    reg [$clog2(EV_FREQ):0] ev_wait_count=0;// evaporate trigger counter
    reg ph_decay_en;
    reg [PH_DATA_SIZE-1:0]decay_ph;
    //outputs :
    
    // decay ph globally in a certain time
    always@(posedge clk)// no rst needed
    begin
    if (ev_wait_count==(EV_FREQ-3))
        begin
        stall<=1;// stalling start , at EV_FREQ start refresh ph, wait 3 cly cycle for last agent
        ev_wait_count<=ev_wait_count+1;
        end
    else if (ev_wait_count==(EV_FREQ+MAP_SIZE*2-1))// two cycles noe for read , one for write back, no pipeline cause has little degration inperformance
        begin
        stall<=0;//finish refresh ph
        ev_wait_count<=0;     
        end
    else
        ev_wait_count<=ev_wait_count+1;
    
    if(ev_wait_count>=EV_FREQ)// refresh ph map
    
        if(ev_wait_count[0]==0)//read phase, calculate address and get ph
            begin
            ph_decay_en<=0;// the address calculation is in the first if of the next block, since it need assign ph_addra
            decay_ph<=ph0;
            end
        else //write phase, decay ph to some degree
            begin
            ph_decay_en<=1;
            decay_ph<= (decay_ph>>1)+(decay_ph>>2) ; // 3/4 of original
            end
    else
        ph_decay_en<=0;
    

    end
    
    
    // position 
    reg web;
    reg [PH_DATA_SIZE-1:0]ph_add;
    
   //decided in the mem read phase which address to choose
   // use [POS_ADDR*2-1:0] pos_active; [1:0] step_active; solved_out; grant_count
   //for ph map:
   // in solved grant_count=2 means read, 1 means write
   //in normal state 3 means first read , 2 means second read, 1 means write 
   
   //if meet edge read start point's map which is 0 indicate an obstacle   
    wire [POS_ADDR-1:0] x = pos_active[POS_ADDR*2-1:POS_ADDR];
    wire [POS_ADDR-1:0] y = pos_active[POS_ADDR-1:0];
    
    wire [POS_ADDR-1:0] x_plus_1 = (x >= (MAP_LEN-1)) ? 0 : x + 1;
    wire [POS_ADDR-1:0] x_minus_1 = (x == 0) ? 0 : x - 1;
    wire [POS_ADDR-1:0] y_plus_1 = (y >=( MAP_LEN-1)) ? 0 : y + 1;
    wire [POS_ADDR-1:0] y_minus_1 = (y == 0) ? 0 : y - 1;
    
    wire [POS_ADDR*2-1:0] address_up = (y == 0) ? 0 : y_minus_1 * MAP_LEN + x;
    wire [POS_ADDR*2-1:0] address_right = (x >= MAP_LEN-1) ? 0 : y * MAP_LEN + x_plus_1;
    wire [POS_ADDR*2-1:0] address_down = (y >= MAP_LEN-1) ? 0 : y_plus_1 * MAP_LEN + x;
    wire [POS_ADDR*2-1:0] address_left = (x == 0) ? 0 : y * MAP_LEN + x_minus_1;
   
   
    always@( * )// no rst needed
    begin
    if(stall && ev_wait_count>=EV_FREQ)
        ph_addra<=EV_FREQ % (MAP_SIZE*2);//value 0-99 in this case
    else if(solved_out && grant_count==1)
        begin
        web=0;
        ph_addrb=pos_active[POS_ADDR-1:0]*MAP_LEN + pos_active[POS_ADDR*2-1:POS_ADDR];
        end
    else if(solved_out && grant_count==0)
        begin
        web<=1;
        ph_add<=ph1+BACK_PR;
        end
    else if(!solved_out && grant_count==2) // first explore    , env and ph
        begin
        web<=0;
        case(step_active)
        2'b00://up move , which means downside of the position would not be considered.
                begin
                map_addra<=address_left;
                map_addrb<=address_up;
                ph_addra<=address_left;
                ph_addrb<=address_up;
                end
        2'b01://right move left not considered
                begin
                map_addra<=address_up;
                map_addrb<=address_right;
                ph_addra<=address_up;
                ph_addrb<=address_right;
                end
        2'b10://down move up not considered
                begin
                map_addra<=address_right;
                map_addrb<=address_down;
                ph_addra<=address_right;
                ph_addrb<=address_down;
                end
        2'b11://left move right not considered
                begin
                map_addra<=address_down;
                map_addrb<=address_left;
                ph_addra<=address_down;
                ph_addrb<=address_left;
                end
        
        endcase
        
        end
    else if(!solved_out && grant_count==1)//second explore
        begin
        web<=0;
        case(step_active)
        2'b00://up move , which means downside of the position would not be considered.
                begin
                map_addra<=address_right;
                ph_addra<=address_right;
                ph_addrb<=pos_active[POS_ADDR-1:0]*MAP_LEN + pos_active[POS_ADDR*2-1:POS_ADDR];
                end
        2'b01://right move left not considered
                begin
                map_addra<=address_down;
                ph_addra<=address_down;
                ph_addrb<=pos_active[POS_ADDR-1:0]*MAP_LEN + pos_active[POS_ADDR*2-1:POS_ADDR];
                end
        2'b10://down move up not considered
                begin
                map_addra<=address_left;
                ph_addra<=address_left;
                ph_addrb<=pos_active[POS_ADDR-1:0]*MAP_LEN + pos_active[POS_ADDR*2-1:POS_ADDR];
                end
        2'b11://left move right not considered
                begin
                map_addra<=address_up;
                ph_addra<=address_up;
                ph_addrb<=pos_active[POS_ADDR-1:0]*MAP_LEN + pos_active[POS_ADDR*2-1:POS_ADDR];
                end
        
        endcase
        
        end
    else if(!solved_out && grant_count==0)// write ph to pos_active
        begin
        web<=1;
        ph_add<=ph1+1;
        ph_addrb<=pos_active[POS_ADDR-1:0]*MAP_LEN + pos_active[POS_ADDR*2-1:POS_ADDR];
        end
   end
    
    
    
    
    //initialize mem blocks
    
    
    map0 map_0(
    .clka(clk),
    .clkb(clk),
    .addra(map_addra),
    .addrb(map_addrb),
    .douta(map_douta),
    .doutb(map_doutb)
    );
    
    blk_mem_gen_0 ph_map(
    .clka(clk),
    .clkb(clk),
    .dina(decay_ph),// when in ph evaporate process use this channel to decrease ph
    .douta(ph0),
    .wea(ph_decay_en),// in stalling, each two cycle enable a refreshed ph write back
    .addra(ph_addra),
    .dinb(ph_add),// read the pos_active's ph and write the self-added value in the grant_value==1 phase
    .doutb(ph1),
    .web(web),
    .addrb(ph_addrb)  
    );
endmodule 