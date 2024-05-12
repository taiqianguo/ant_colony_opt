`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 07.05.2024 18:28:50
// Design Name: 
// Module Name: ant_agent
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


module ant_agent#(parameter RSEED =1)(
    input clk ,
    input rst,
    output solved_out,// solved go back process add more ph during back.
//environment observation
    input bus_av,//ask if need to read map.
    output reg bus_request=1,// 
//environment observation
    output reg [POS_ADDR*2-1:0]pos_now=0,
    output reg [1:0]step,// previous step,00 up, 01 right,10 down, 11 left // previous moves' step
    input data0_map,
    input data1_map,
    
//pheronmone observation and write
    input [PH_DATA_SIZE-1:0]ph0,
    input [PH_DATA_SIZE-1:0]ph1
    );
    
    
    parameter MAP_SIZE=100;//square of map length  k^2
    parameter  MAP_LEN=10;
    parameter POS_ADDR=4;// 10<2^4
    
    parameter MAX_LEN=100;// max search length of ant after that ant reintialized to origianl point.
    parameter STACK_ADDR=7; //128 address space larger than 100
    parameter pos_de=8'b10011001;// the destination (9,9)
    parameter PH_DATA_SIZE=16;
    
    parameter ALPHA=2; // left shift 2 , a four times coefficient
    parameter BELTA=13'b1000000000000; //offset of random selection
    
    
    
    
    //control regs for g_state and d_state
    reg solved=0;// 1 indicate this agent got a path and is returning to origion.
    assign solved_out=solved;
    reg stack_wen;// 1 indicate decision next step process is finished and write previous step to stack, also indicate state transition of g_state.
    reg [STACK_ADDR-1:0]stack_pointer; // the stack pointer to trace the path 
    
    
    
    // global state machine for control,
    reg [1:0] g_state=2'b00;// 00 wait memory handle, 01 observe env ph and write ph, 10 decide next setp 
    reg [1:0] d_state=2'b00;//00 idle(finished), 01 check solution, 10 random selection and write back
    
    always@(posedge clk )
    if (!rst)
        g_state<=00;
    else
        case(g_state)
            2'b00:
                if (!bus_av) g_state<=g_state;// stay idle waiting for mem_handle
                else g_state<=2'b01;// observe env and ph 
            2'b01:// memory access phase
                if(bus_av) g_state<=g_state;// observe not finish , stay in state
                else g_state<=2'b10;
            2'b10:// decision making phase
                if (d_state==2'b00) g_state<=2'b00;// step decision process ended.
                else g_state<=g_state;
            default g_state<=2'b00;
      
        endcase 
    
    //decision state 
    always@(posedge clk )
    if (!rst)
        begin
        d_state<=2'b00;
        solved<=0;
        end
    else if(stack_pointer==0 && solved==1) //go back process finished
        begin
        solved<=0;
        d_state<=0;
        end
    else
        case(d_state)
        2'b00:
            if(g_state==2'b01 && !bus_av) d_state<=2'b01;// observe finished get into first work state of d_state
            else d_state<=d_state;
        2'b01:// this state is for  step aqusition from stack for soloved
            if(pos_now==pos_de) //reached the destination
                begin 
                    solved<=1;
                    d_state<=2'b10;// start solved process, get the trace_step as the next step 
                end
            else if (solved==1 && stack_pointer!=0) //go back process, get the trace_step as the next step 
                    d_state<=2'b10;
            else if(stack_pointer==(MAX_LEN-1))// get env reached the max search lenght of ant , restart from original
                    d_state<=2'b00;
            else if(av_path==0) //get env ,cornercase ant reintialized to origin.  set pos and stack_pointer to 0 wait for next start
                    d_state<=2'b00;
            else d_state<=2'b10;   // normal explore process, get next possible steps to p012 
        2'b10: // here solved and normal explore get this state, solved will directly use the next step update the position and retun to 00 state
            if(solved==1) d_state<=2'b00;// one cycle for position update
            else if (stack_wen) d_state<=2'b00;//in random selection process after stack write enabled 
            else d_state<=d_state;   
        2'b11:
            d_state<=2'b00;
    
        endcase
     
     
     
     
     //env observation and  men control
     reg [18:0]p0=0;// the LSB is a path flag if is set 1,then two bits indicate direction of next steps, the the rest shows ph value. which is 16+2+1
     reg [18:0]p1=0;
     reg [18:0]p2=0;
     wire av_path=(p2[0] || p1[0] || p0[0]) ;// at least 1 path is 1 which is feasibale
     
     always@(posedge clk )
     if (!rst)
            begin
            bus_request<=0;
            end
     else if (g_state==2'b00) // when finished process in dile state request next mem handle
        bus_request<=1;       // controller will give two clk cycle to this request and leave.
     else bus_request<=0;   
        
        
     reg [1:0]mem_cycle=2;// seperate control of not read, first read and second read
     always@(posedge clk )// in clock direction read first two position's obstacle and ph
     if (!rst)
        begin
        mem_cycle<=2;
        end
     else if (solved==0 && bus_av && mem_cycle==2) // if not solved, in cycle 1 recieve data of p0,p1 , 
        begin
        mem_cycle<=mem_cycle-1;
        p0<=p0;
        p1<=p1;
        p2<=p2;
        end
     else if(mem_cycle==1)// the second mem cycle 
        begin
        mem_cycle<=0;
        p0[18:3]<=ph0;
        p1[18:3]<=ph1;
        p0[0]<=data0_map;
        p1[0]<=data1_map;
        end
     else if(mem_cycle==0) 
        begin
        mem_cycle<=2;
        p2[18:3]<=ph0;
        p2[0]<=data0_map;
        end
     else
         case(step)//after mem access finished , calculate next available step based on previous executed step , here no check for boundaries, since if outsdie the boundary, other functions would 
                        // prevent it from execution, this step is executed with d_state01 in parallel
            2'b00://up move , which means downside of the position would not be considered.
                begin
                p0[2:1]<=2'b11 ;//left
                p1[2:1]<=2'b00;//up
                p2[2:1]<=2'b01;//right
                end
            2'b01://right move left not considered
                begin
                p0[2:1]<=2'b00;//up
                p1[2:1]<=2'b01;//right
                p2[2:1]<=2'b10;//down
                end
            2'b10://down move up not considered
                begin
                p0[2:1]<=2'b01;//right
                p1[2:1]<=2'b10;//down
                p2[2:1]<=2'b11;//left
                end
            2'b11://left move right not considered
                begin
                p0[2:1]<=2'b10;//down
                p1[2:1]<=2'b11;//left
                p2[2:1]<=2'b00;//up
                end
     endcase
     
     
     
     //random generator
    reg [47:0] random_number=48'hFFFF_FFFF_FFFF-RSEED;
    wire feedback = random_number[47] ^ random_number[27] ^ random_number[26] ^ random_number[0];

    always @(posedge clk )
        begin
        if (!rst) 
            random_number <= 48'hFFFF_FFFF_FFFF-RSEED; // Set to a non-zero initial state
        else 
            random_number <= {random_number[46:0], feedback};
        end
     
     

     
     //stack control and step move in d_state=2'b01 which is only 1 clk cycle
     wire [1:0]trace_step;// in solved condition assign next step to this trace 
     reg [1:0]next_step=0;// a temp
     reg next_get_flag=0;// a flag indicate get next and can trigger stack_wen and update pos process
     
     reg [15:0] max_ph=0;
     reg [1:0]rnd=0;
     stacks stack(
     .addra(stack_pointer),
     .clka(clk),
     .dina(next_step),
     .douta(trace_step),
     .wea(stack_wen)// this write en trigger's state machine finish a round of process and transfer to idle
     );
     
     always @(posedge clk ) 
     if(!rst )
        begin
        stack_pointer<=0;
        pos_now<=0;
        step<=2'b10;
        stack_wen<=0;
        end
     else if (d_state==2'b01)// first  stack pointer, then next_step the position
        begin
            if(pos_now==pos_de ) //detected reached the destination, next pos is stack_pointer's data
                stack_pointer<=stack_pointer-1;
            else if (solved==1 && pos_now!=0) //go back process
                stack_pointer<=stack_pointer-1;
            else if(solved==1 && pos_now==0)
                step<=2'b10;
            else if(stack_pointer==(MAX_LEN-1))// reached the max search lenght of ant , restart from original set solved=0 in state machine
                begin
                    pos_now<=0;
                    stack_pointer<=0;
                    step<=2'b10;
                end
            else if(av_path==0) //cornercase ant reintialized to origin.
                 begin
                    pos_now<=0;
                    stack_pointer<=0;
                    step<=2'b10;
                 end
            else stack_pointer<=stack_pointer+1;
        end
     else if(d_state==2'b10)
        begin
            max_ph<=0;
            if(pos_now!=0 && solved)
                begin // after pointer subtracted and not solved, sync next step to the trace
                //here we get trace_step to solved trace back , THE OPPOSITE DIRECTION!!!
                case(trace_step)// since only 1 clk cycle for this solved in d_state10 only execute once
                    2'b00:pos_now<=pos_now+1;//00 is up so we move down
                    2'b01:pos_now[POS_ADDR*2-1:POS_ADDR]<=pos_now[POS_ADDR*2-1:POS_ADDR]-1;//right move left
                    2'b10:pos_now<=pos_now-1;//10 down move up
                    2'b11:pos_now[POS_ADDR*2-1:POS_ADDR]<=pos_now[POS_ADDR*2-1:POS_ADDR]+1;//move right
                endcase
                end
                
            else // normal explorationa verilog function that three number with valid flags ,  
                        //those valid numbers need to compare with random value, if all valid numbers smaller than random number, then randomly select a valid number .
                         // If not aselect the largest valid number. 
            begin
                if(next_get_flag==0)
                    begin
                    next_get_flag<=1;
                    // Determine the maximum of the valid numbers
                    if (p0[0] && p0[18:3] > max_ph)
                        begin
                        max_ph <= p0[18:3];
                        next_step<=p0[2:1];
                        end
                    if (p1[0] && p1[18:3] > max_ph)
                        begin
                        max_ph <= p1[18:3];
                        next_step<=p1[2:1];
                        end                     
                    if (p2[0] && p2[18:3] > max_ph)
                        begin
                        max_ph <= p2[18:3];
                        next_step<=p2[2:1];
                        end
                    // Check if all valid numbers are smaller than the random value
                    
                    if ((p0[0]==0 ||  (BELTA+p0[18:3]) < random_number[15:0]) && (p1[0]==0 || (BELTA+p1[18:3]) < random_number[31:16]) && (p2[0]==0 || (BELTA+p2[18:3]) < random_number[47:32])) 
                        // If true, select a random valid number
                        begin
                        rnd <= random_number % 3; // mod Random number between 0 and 2
                        case (rnd)
                            0: next_step <= p0[0] ? p0[2:1] : (p1[0] ? p1[2:1] : p2[2:1]);
                            1: next_step <= p1[0] ? p1[2:1] : (p2[0] ? p2[2:1] : p0[2:1]);
                            2: next_step <= p2[0] ? p2[2:1] : (p0[0] ? p0[2:1] : p1[2:1]);
                        endcase
                        next_get_flag<=1;
                        stack_wen<=1;
                        end
                    else 
                       begin
                        // Otherwise, select the largest valid number
                       next_get_flag<=1;
                       stack_wen<=1;
                       end
    
                    end
                else // update pos_now and write stack in parallel   
                    begin
                    case(next_step)//update next step
                    2'b00:pos_now<=pos_now-1;//up move
                    2'b01:pos_now[POS_ADDR*2-1:POS_ADDR]<=pos_now[POS_ADDR*2-1:POS_ADDR]+1;//move right
                    2'b10:pos_now<=pos_now+1;
                    2'b11:pos_now[POS_ADDR*2-1:POS_ADDR]<=pos_now[POS_ADDR*2-1:POS_ADDR]-1;// move left
                    endcase
                    next_get_flag<=0;
                    step<=next_step;// all process finish and g_state to idle
                    stack_wen<=0;//in the last state of d_state10 close write enable 
                    end
           end
       end
     else  ;   
endmodule 