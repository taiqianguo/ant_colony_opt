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

//first detect request, if solved flag ,only wait 1 cycle and wirte ph to position
    // else  set bus_av to 1 at the same time calculate first two address
    //then calculate the third address , pass bua_av to next agent
    // transfer the pos_now and step_pre to data of env and obstiacles
module controller(
    input clk,
    input rst,
    input stall,
    input [NUM_AGENTS-1:0]solved,
    input [NUM_AGENTS-1:0] bus_request,
    output reg [NUM_AGENTS-1:0] bus_av,
    input [NUM_AGENTS*POS_ADDR*2-1:0] poses_now, // Input for all agents' memory addresses
    output reg [POS_ADDR*2-1:0] pos_active,
    input [NUM_AGENTS*2-1:0]steps,
    output reg [1:0] step_active,
    output reg solved_out,
    output reg [1:0] grant_count // in solved==0,  grant_count==1 , channel b write ph to ph_map
);
    parameter NUM_AGENTS = 8;
    parameter POS_ADDR = 4;
    parameter MAP_LEN =10;


    reg [$clog2(NUM_AGENTS)-1:0] current_agent;
    
    always @(posedge clk) 
    begin

        if (!rst) 
            begin
            bus_av <= 0;
            pos_active <= 0;
            step_active<=0;
            current_agent <= 0;
            grant_count <= 0;
            end
        else if (grant_count==0 && stall) // stall the controller for global ph update , only after finish one access can stall occur
            begin
            grant_count<=grant_count;
            bus_av<=0;
            current_agent<=current_agent;
            end
        else 
            if (grant_count > 0) 
            begin
                grant_count <= grant_count - 1;
                pos_active <= pos_active;
                step_active<=step_active;
            end 
            else 
            begin
            current_agent<=(current_agent+1)% NUM_AGENTS; // a in trun check
            if (bus_request[current_agent]) 
                begin
                bus_av <= (1 << current_agent);
                pos_active <= poses_now[current_agent*POS_ADDR*2 +: POS_ADDR*2];
                step_active<=steps[current_agent * 2 +: 2];
                // Check if the agent is solved and assign grant_count accordingly
                if (solved[current_agent] == 1'b1) 
                    begin
                    grant_count <= 1; // Only grant for 2 cycle if solved
                    solved_out<=1;
                    end
                else 
                    begin
                    grant_count <= 2; // Otherwise grant for 2 cycle   , in 2 cycle channel 2 read current position ph and add a number in cycle3 write back
                    solved_out<=0;
                    end                 
                end
                
            end
        end

endmodule