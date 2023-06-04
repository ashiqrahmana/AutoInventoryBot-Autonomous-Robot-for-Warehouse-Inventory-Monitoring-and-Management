#include "simpletools.h"                      // Include simple tools
#include "servo.h"                            // Include servo header
#include "time.h"
const int left   = 13;
const int ctr    = 14;
const int right  = 11;
const int intersection_led = 4;
float mid_l,mid_c,mid_r;
int left_ir,right_ir,center_ir;

const int leftServo = 17;
const int rightServo = 16;
int wheel_speed = 10;

int dist[2] = {1000,1000};

const int ultra[2][2] = {{11,10},{0,1}};

int currPos[2] = {1,-1};  
int ultiEnd[2] = {0,3}; 
int startPos[2] = {1,0}; 
int endPos[2] = {0,4}; 
int prevPos[2] = {1,0}; 

serial *lcd;
const int ON = 22;
const int CLR = 12;
int phi = 0;
int uTurn_flag = 0;
int status = 0;
int PATH[100][2] = {{1,-1},{1,0}};
int pathSize=2;
int segment = 1;

//---------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------
//----------------------------------------A star Planner-------------------------------------------
//---------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define ROWS 3
#define COLS 7

typedef struct {
    int x, y;
} Point;

typedef struct {
    Point parent;
    Point location;
    double f, g, h;
} Node;

int grid[ROWS][COLS] = {{0,0,0,0,0,0,0},
                        {0,1,0,1,0,1,0},
                        {0,0,0,0,0,0,0}};

Node openList[ROWS*COLS];
Node pathList[ROWS*COLS];
int openListCount = 0;
int pathListCount = 0;
int closedList[ROWS][COLS];

Point startPoint ;
Point endPoint;

void addToOpenList(Node n) {
    openList[openListCount] = n;
    openListCount++;
}

void addToPathList(Node n) {
    pathList[pathListCount] = n;
    pathListCount++;
}

int isPointInClosedList(Point p) {
    return closedList[p.x][p.y];
}

int getIndexOfLowestFCost() {
    int index = 0;
    double lowestCost = openList[0].f;
    for (int i = 1; i < openListCount; i++) {
        if (openList[i].f < lowestCost) {
            lowestCost = openList[i].f;
            index = i;
        }
    }
    return index;
}

void removeFromOpenList(int index) {
    for (int i = index; i < openListCount - 1; i++) {
        openList[i] = openList[i+1];
    }
    openListCount--;
}

double heuristic(Point p1, Point p2) {
    return sqrt(pow((p2.x - p1.x), 2) + pow((p2.y - p1.y), 2));
}

int isPointInOpenList(Point p) {
    for (int i = 0; i < openListCount; i++) {
        if (openList[i].location.x == p.x && openList[i].location.y == p.y) {
            return 1;
        }
    }
    return 0;
}

int findIndex(Point p) {
    for (int i = 0; i < pathListCount; i++) {
        if (pathList[i].location.x == p.x && pathList[i].location.y == p.y) {
            return i;
        }
    }
    return -1;
}

double gCost = 0;
double hCost = 0;
double fCost = 0; 

void aStar() {
    Node currentNode = {{0,-1},{startPoint.x, startPoint.y}, 0.0, 0.0, 0.0};
    addToOpenList(currentNode);
    while (openListCount > 0) {
        //printf("ok wow \n");
        int currentIndex = getIndexOfLowestFCost();
        currentNode = openList[currentIndex];
        
        if (currentNode.location.x == endPoint.x && currentNode.location.y == endPoint.y) {
            //printPath(currentNode);
            return;
        }
        removeFromOpenList(currentIndex);
        closedList[currentNode.location.x][currentNode.location.y] = 1;
        
        Point possible_nodes[3];
        int possible_node_size = 0;
        int node_cost;
        if (currentNode.location.x==2){
            possible_nodes[0] = (Point){0,-1};
            possible_nodes[1] = (Point){-1,0};
            possible_nodes[2] = (Point){0,1};
            possible_node_size = 3;  
        }
        else if (currentNode.location.x==1){
            possible_nodes[0] = (Point){-1,0};
            possible_nodes[1] = (Point){1,0};
            possible_nodes[2] = (Point){0,1};
            possible_node_size = 3;  
        }  
        else if (currentNode.location.x==0){
            possible_nodes[0] = (Point){0,-1};
            possible_nodes[1] = (Point){1,0};
            possible_nodes[2] = (Point){0,1};
            possible_node_size = 3;  
        } 
        
        for(int i = 0; i < possible_node_size; i++)
        {
            int x = currentNode.location.x + possible_nodes[i].x;
            int y = currentNode.location.y + possible_nodes[i].y;
            
            if (x == 0 || x == 2){
                node_cost = 0;
            }
            else {
                node_cost = 0;
            }
            
            if (x < 0 || x >= ROWS || y < 0 || y >= COLS) {
                continue;
            }
            if (isPointInClosedList((Point){x, y})) {
                continue;
            }
            if (grid[x][y] == 1){
                continue;
            }
            
            //printf("%d , %d \n", x, y);
            
            gCost = currentNode.g + 1.0;
            hCost = heuristic((Point){x, y}, endPoint)+node_cost;
            fCost = gCost + hCost;
            
            if (!isPointInOpenList((Point){x, y})) {
                /*
                printf("Node: Parent - %d ,%d |  ",currentNode.location.x,currentNode.location.y);
                printf("      Child - %d ,%d |  \n",x,y);
                printf("      g - %lf |  ",gCost);
                printf("      h - %lf |  ",hCost);
                printf("      f - %lf |  \n",fCost);*/
                addToPathList((Node){{currentNode.location.x,currentNode.location.y},{x, y}, fCost, gCost, hCost});
                addToOpenList((Node){{currentNode.location.x,currentNode.location.y},{x, y}, fCost, gCost, hCost});
            }
            
            else {
                int index = closedList[x][y];
                if (gCost < openList[index].g) {
                    openList[index].f = fCost;
                    openList[index].g = gCost;
                    openList[index].h = hCost;
                    openList[index].parent = (Point){currentNode.location.x, currentNode.location.y};
                }
            }
            
        }
    }
}

void init(){
    for (int i=0; i<ROWS*COLS; i++){
        openList[i] = (Node){{-1,-1},{0, 0}, 0, 0, 0};
        pathList[i] = (Node){{-1,-1},{0, 0}, 0, 0, 0};        
    }
    openListCount = 0;
    pathListCount = 0;
    gCost = 0;
    hCost = 0;
    fCost = 0;
    for (int i=0; i<ROWS; i++){
        for (int j=0; j<COLS; j++){
            closedList[i][j] = 0;
        }
    }
}

void path_planner(int startX, int startY, int endX, int endY)
{
        
        //print("Path_planner called\t\n");
        startPoint.x = startX;
        startPoint.y = startY;
        endPoint.x = endX;
        endPoint.y = endY;
        
        printf("%d ,%d | %d ,%d\n",startPoint.x,startPoint.y,endPoint.x,endPoint.y);
        
        init();
        
        aStar();
        Point buildPath[10];
        int index = 0;
        index = findIndex(endPoint);
        /*
        printf("%d\n",index);
        printf("Node: Parent - %d ,%d |  ",pathList[index].parent.x,pathList[index].parent.y);
        printf("      Child - %d ,%d | \n",pathList[index].location.x,pathList[index].location.y);
        */
        int k=0;
        
        for(int i = 0; i< 9;i++){
            if (index==-1){
                //buildPath[i] = startPoint;
                break;
            }
            buildPath[i] = pathList[index].location;
            k=i;
            //printf("%d,%d \n",buildPath[i].x,buildPath[i].y);
            if (pathList[index].parent.x == startPoint.x && pathList[index].parent.y == startPoint.y){
                buildPath[i] = pathList[index].location;
                buildPath[i+1] = pathList[index].parent;
                //printf("%d,%d \n",buildPath[i].x,buildPath[i].y);
                //printf("%d,%d \n",buildPath[i+1].x,buildPath[i+1].y);
                break;
            }
            index = findIndex(pathList[index].parent);
            //printf("Index: %d \n",index);
        }
        Point path[k+1];
        pathSize=0;
    
        for (int j=k+1; j>=0; j--){
            path[pathSize]=buildPath[j];
            PATH[pathSize][0] = path[pathSize].x;
            PATH[pathSize][1] = path[pathSize].y;
            printf("%d,%d \n",PATH[pathSize][0],PATH[pathSize][1]);
            pathSize++;
        }/*
        pathSize++;
        PATH[pathSize][0] = 1000;
        PATH[pathSize][1] = 1000;
        //printf("%d,%d \n",PATH[pathSize][0],PATH[pathSize][1]);*/
      
}


//---------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------
//----------------------------------------A star Navigator-------------------------------------------
//---------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------

void display(char string[],int value){
  lcd = serial_open(7, 7, 0, 9600);
  writeChar(lcd, ON);
  writeChar(lcd, CLR);
  pause(5);
  if(value == -1){
   dprint(lcd, string);  
  }  
  else{
    dprint(lcd, string, value);
  }    
}  

void get_distance_side(int trigPin, int echoPin,int index)
{
   low(trigPin);
   pulse_out(trigPin,5);
   long tEcho = pulse_in(echoPin,1);
   dist[index] = tEcho/58;
   //display("Distance = %d\n",dist[index]);
   printf("Distance = %d | %d\n",dist[0], dist[1]);
}

long RCTime(int sensorIn){
  long duration = 0;
  
  set_direction(sensorIn,1);// Sets pin as OUTPUT
  high(sensorIn); 
  pause(1);                        // Waits for 1 milli second
  set_direction(sensorIn,0);// Sets pin as INPUT
  
  while(input(sensorIn)) {    // Waits for the pin to go LOW
    duration++;
  }
  return duration ;                 // Returns the duration of the pulse
}  

void callibration() {
  display("White enter ok\n",-1);
  printf("Place all the sensors in white, Enter ok to continue\n");
  int button = 8;
  set_direction(button,0);
  float low_l,low_r,low_c,high_l,high_c,high_r;
  
  while (1) {
    if (input(button) == 1) {
      low_l = RCTime(left);
      low_r = RCTime(right);
      low_c = RCTime(ctr);
      break;
    }
  }
  pause(1000);
  display("Black enter ok\n",-1);
  printf("Place all the sensors in black, Enter ok to continue\n");
  while (1) {
    if (input(button) == 1) {
      high_l = RCTime(left);
      high_r = RCTime(right);
      high_c = RCTime(ctr);
      break;
    }
  }

  mid_l = (low_l + high_l) / 4;
  mid_c = (low_c + high_c) / 5;
  mid_r = (low_r + high_r) / 4;
}

void ir_read() {
  
  if (RCTime(left) > mid_l) {
  left_ir = 1;
  } else {
  left_ir = 0;
  }
  
  if (RCTime(right) > mid_r) {
  right_ir = 1;
  } else {
  right_ir = 0;
  }
  
  if (RCTime(ctr) > mid_c) {
  center_ir = 1;
  } else {
  center_ir = 0;
  }
  print("cp=(%d,%d),cp(path)=(%d,%d),phi=%d,seg=%d,ps=%d,uf=%d,l=%d,c=%d,r=%d \n",currPos[0],currPos[1],PATH[0][0],PATH[0][1],phi,segment,pathSize,uTurn_flag,left_ir,center_ir,right_ir);
    //for (int i = 0; i < pathSize; i++) {
    //    print("%d,%d \t",PATH[i][0],PATH[i][1]);
    //}
    //print("\t");
}

void left_turn() {
  // turn back to the original position
  int prev_speed = wheel_speed;
  wheel_speed = 100;
  //display("WS: %d",wheel_speed);
  servo_angle(leftServo,900 - wheel_speed);
  servo_angle(rightServo,900 + wheel_speed);
  pause(250);
  servo_angle(leftServo,900 - wheel_speed);
  servo_angle(rightServo,900 - wheel_speed);
  printf("turning left\n");
  pause(500);
  phi += -90;
  wheel_speed = prev_speed;
  //pause(3000);
}

void right_turn(){
  // turn back to the original position
  int prev_speed = wheel_speed;
  wheel_speed = 100;
  //display("WS: %d",wheel_speed);
  servo_angle(leftServo,900 - wheel_speed);
  servo_angle(rightServo,900 + wheel_speed);
  pause(250);
  servo_angle(leftServo,900 + wheel_speed);
  servo_angle(rightServo,900 + wheel_speed);
  printf("turning right\n");
  pause(500);
  phi += 90;
  wheel_speed = prev_speed;
  //pause(3000);
}  

int seg3_flag=0;
int distance = 0;

int readRPi(){
  int return_var = 0;
  while(1){
   if(input(1) == 1){
    return_var = 1;
    break;
   }
   else if(input(2) == 1){
    return_var = 2;
    break;
  }    
   else{
    continue;
   }      
  } 
   return(return_var);  
}

int return_pi = 0;
int newFlag = 0;
int right_flag = 1;
int special = 0;

void no_mans_land(int direction){
  int falling_edge = 0;
  int multiplier = 1;
  while(input(0)==1){
      // Set Dir for rotation
      if (direction == 1){  // left
      wheel_speed = -20;
      multiplier = 1;              //Control front motion in drive for both cases.
      high(3);
      }
      else if (direction == 2){   //right
      wheel_speed = +20;
      multiplier = -1;
      low(3);
      }        
      // Rotate
      servo_angle(leftServo,900 - wheel_speed);
      servo_angle(rightServo,900 - wheel_speed);
      printf("Adjusting\n");
      pause(40);
    }
  if(input(0)==0 && input(12) == 0){
      //Drive straight and check again
      servo_angle(leftServo,900 - abs(wheel_speed));
      servo_angle(rightServo,900 + abs(wheel_speed));
      printf("Drive\n");
      pause(40);
      //Insert if condition to stop. based on ir sensors
      } 
      ir_read();
      while(center_ir == 1){  
        ir_read();
        servo_angle(leftServo,900);
        servo_angle(rightServo,900); 
      }
  servo_angle(leftServo,900);
  servo_angle(rightServo,900);
}  

void bot_align(int direction){
   if(direction == 1){ //left
      wheel_speed = 30;
      servo_angle(leftServo,900 + wheel_speed);
      servo_angle(rightServo,900 + wheel_speed);
      pause(1000);
      servo_angle(leftServo,900);
      servo_angle(rightServo,900);
      pause(1000);
      wheel_speed = 10;
   }
   else if (direction ==2){ //right
     wheel_speed = 30;
      servo_angle(leftServo,900 - wheel_speed);
      servo_angle(rightServo,900 - wheel_speed);
      pause(1000);
      servo_angle(leftServo,900);
      servo_angle(rightServo,900);
      pause(1000);
      wheel_speed = 10;
   }      
}

int no_mans_lalaland()                                    // Main function
{
  // Add startup code here.
  print("Lets Goooo!\n");
  servo_angle(leftServo,900);
  servo_angle(rightServo,900);
  int return_pi = readRPi();
  servo_angle(leftServo,900 - wheel_speed);
  servo_angle(rightServo,900 +  wheel_speed*1.5);
  pause(1000);
  servo_angle(leftServo,900);
  servo_angle(rightServo,900);    
  bot_align(return_pi);
  while(1){   
   //print("Start No Man's land\n");
   no_mans_land(return_pi);
  }
}

void basic_line_follower(){
  ir_read();
  
  if (currPos[1] == 5 && left_ir == 0 && right_ir == 0 && center_ir == 0){
    low(15);
    no_mans_lalaland();
  }     
  if (left_ir == 1 && right_ir == 1 && center_ir == 1  || special == 1 ) {
    special = 0;
    display("Intersection Detected",-1);
    printf("Intersection Detected");
    if ((phi == 0 && currPos[0] == 1) || (right_flag == 1 && phi == 90 && currPos[0] == 0) || (right_flag == 0 && phi == -90 && currPos[0] == 2)){
      high(intersection_led);
    }    
    // Dont touch this 
    int x=(PATH[1][0] - PATH[0][0]);
    int y=(PATH[1][1] - PATH[0][1]);
    if (phi == 0){
      if (x==-1 && y==0){
        currPos[0] -= 1;
      }        
      else if(x==1 && y==0){
        currPos[0] += 1;
      }        
      else{
       currPos[1] += 1;
      }      
    }      
    else if (phi == 90){
      if (x==0 && y==-1){
        currPos[1] -= 1;
      }        
      else if(x==0 && y==1){
        currPos[1] += 1;
      }     
      else{
        currPos[0] += 1;
      }    
    }      
    else if (phi == -90){
      if (x==0 && y==-1){
        currPos[1] -= 1;
      }        
      else if(x==0 && y==1){
        currPos[1] += 1;
      }     
      else{
        currPos[0] -= 1;
      }   
    }      
    else if (phi == 180){     
      if (x==-1 && y==0){
        currPos[0] -= 1;
      }        
      else if(x==1 && y==0){
        currPos[0] += 1;
      }        
      else{
        currPos[1] -= 1;
      }      
    }   
    
    startPos[0] = currPos[0];   
    startPos[1] = currPos[1];
    //display("Intersection : %d\n",startPos[1]);
    printf("Intersection : %d\n",startPos[1]);
    
    if (pathSize<=2){  
      segment++;
      newFlag = 1;
      if (segment%3 == 0){
        grid[currPos[0]+1][currPos[1]] = 0;
      }
    }         
    printf("****************************%d \n ",segment);
    if(segment%3 == 1){
      printf("------------------------------%d \n ",newFlag);
      
      if(currPos[0] == 1 && newFlag ==1 && phi==0){
        servo_angle(leftServo,900);
        servo_angle(rightServo,900);
        return_pi = readRPi();
        if(return_pi == 1){ // reading blue arrow - HIGH - RIGHT
          endPos[0] = currPos[0] + 1;
          endPos[1] = currPos[1] + 1;
          right_flag = 1;
        }        
        else if(return_pi == 2){ // reading blue arrow - HIGH - LEFT
          endPos[0] = currPos[0] - 1;
          endPos[1] = currPos[1] + 1;
          right_flag = 0;
        }             
      }      
      
      newFlag = 0;  
      if (right_flag == 1){
        high(3);
      }
      else{
        low(3);
      }                          
    }
    else if(segment%3 == 2){
      
      if (right_flag == 1 && newFlag == 1){
        endPos[0] = 0;
        endPos[1] = currPos[1];      
      }        
      else if (right_flag == 0 && newFlag == 1){
        endPos[0] = 2;
        endPos[1] = currPos[1];      
      }  
      if(newFlag == 1){      
        int prev_speed = wheel_speed;
        wheel_speed = 100;
        servo_angle(leftServo,900 + wheel_speed);
        servo_angle(rightServo,900 +  wheel_speed);
        display("Making a Uturn\n",-1); 
        phi += 180;
        printf("newflag is one!!! paused for 10 sec");
        pause(1100);
        wheel_speed = prev_speed;
      }        
      newFlag = 0;
        
      if (right_flag == 1){
        low(3);
      }
      else{
        high(3);
      }                    
    }
    else if(segment%3 == 0){
      if(newFlag == 1 && right_flag == 1){     
        endPos[0] = currPos[0] + 1;
        endPos[1] = currPos[1] + 1;
        grid[1][currPos[1]] = 0;      
      }
      else if(newFlag == 1 && right_flag == 0){     
        endPos[0] = currPos[0] - 1;
        endPos[1] = currPos[1] + 1;
        grid[1][currPos[1]] = 0;      
      }
      newFlag = 0;
                  
    }
    
    path_planner(startPos[0],startPos[1],endPos[0],endPos[1]); 
    
    // dont touch from here 
    x=(PATH[1][0] - PATH[0][0]);
    y=(PATH[1][1] - PATH[0][1]);
    
    if (phi == 0){
      if (x == 1 && y == 0){
        right_turn();
      }        
      else if (x == -1 && y == 0){
        left_turn();
      }           
      else{    
        int prev_wheel_speed = wheel_speed;
        wheel_speed = 100;
        servo_angle(leftServo,900 - wheel_speed);
        servo_angle(rightServo,900 +  wheel_speed);
        //display("slightly front\n",-1);
        pause(200);    
        wheel_speed = prev_wheel_speed;
      }          
    }
    
    else if (phi == 180){
      if  (x == -1 && y == 0){
        right_turn();
      }        
      else if (x == 1 && y == 0){
        left_turn();
      }           
      else{    
        int prev_wheel_speed = wheel_speed;
        wheel_speed = 100;
        servo_angle(leftServo,900 - wheel_speed);
        servo_angle(rightServo,900 +  wheel_speed);
        //display("slightly front\n",-1);
        pause(200);    
        wheel_speed = prev_wheel_speed;
      }             
    }
    
    else if (phi == 90){
      {        
        if (x == 0 && y == -1){
          right_turn();
        }        
        else if (x == 0 && y == 1){
          left_turn();
        }           
        else{    
          int prev_wheel_speed = wheel_speed;
          wheel_speed = 100;
          servo_angle(leftServo,900 - wheel_speed);
          servo_angle(rightServo,900 +  wheel_speed);
          pause(200);    
          wheel_speed = prev_wheel_speed; 
        }    
      }                   
    }
    
    else if (phi == -90){
      if (x == 0 && y == -1){
        left_turn();
      }        
      else if (x == 0 && y == 1){
        right_turn();
      }             
      else{    
      
        int prev_wheel_speed = wheel_speed;
        wheel_speed = 100;
        servo_angle(leftServo,900 - wheel_speed);
        servo_angle(rightServo,900 +  wheel_speed);
        pause(200);    
        wheel_speed = prev_wheel_speed;  
      }      
    }  
    display(" ",-1); 
  }
  
  if (phi > 180){
    phi = -90;
  }
  else if(phi < -90){
    phi = 0;
  }    
  
  // till here
  
  // Dont touch from here     
  else if (center_ir == 1 && left_ir == 0 && right_ir == 0) {
    // straight
    servo_angle(leftServo,900 - wheel_speed);
    servo_angle(rightServo,900 +  wheel_speed*1.5);
    //display("front\n",-1);
    low(intersection_led);                                  // Set P26 I/O pin 

  }
  else if (left_ir == 1) {  
    // correction left
    servo_angle(leftServo,900 - wheel_speed*2);
    servo_angle(rightServo,900 + wheel_speed*1.5);   
    //print("correction left\n");
    //display("correction left\n",-1);
    low(intersection_led);                                  // Set P26 I/O pin low
  }
  else if (right_ir == 1) {
    // correction right
    servo_angle(leftServo,900 - wheel_speed);
    servo_angle(rightServo,900 + wheel_speed*1.5*2);
    //print("correction right\n");
    //display("correction right\n",-1);
    low(intersection_led);                                  // Set P26 I/O pin low
  }
  else {
    // stops the motor out of minor corrections
    servo_angle(leftServo,900);
    servo_angle(rightServo,900);
    //print("stop\n");
    //display("stop\n",-1);
    low(intersection_led);                                  // Set P26 I/O pin low
  }
  
}  

void mazeUpdate(int status){
  if (status == 1){
     grid[PATH[2][0]][PATH[2][1]] = 1;
     for (int i=0; i<ROWS; i++){
          for (int j=0; j<COLS; j++){
              //printf("%d ",grid[i][j]);
          }
          //printf("\n");
      }
  }
  else{
   for (int i=0; i<ROWS; i++){
        for (int j=0; j<COLS; j++){
            grid[i][j] = 0;
            //printf("%d ",grid[i][j]);
        }
        //printf("\n");
    }
  }    
}  

void acquire_target(){
  display("select the target location\n",-1);
  //printf("Place all the sensors in white, Enter ok to continue\n");
  int target_button = 0;
  int confirm_button = 8;
  ultiEnd[1] = 0;
  while(input(confirm_button) != 1){
    if (input(target_button) == 1){
      ultiEnd[1] += 1;
      display("target location = %d\n",ultiEnd[1]);
      pause(500);
    }      
  } 
  display("target location selected\n",-1);
  pause(1000);   
}
int count_stop = 0;  

// pin 3 to control the range of servo left and right
// pin 15 to fix servo at 90

void main()                          // Main function
{ 
 //display("Initiating callibration!!",-1);
 pause(100);  
 callibration();
 printf("Done callibration");
 display("Done callibration",-1); 
 set_direction(leftServo,1);
 set_direction(rightServo,1);
 pause(5000);
 //display(" ",-1);
 segment = 0; 
 while (1){
    if((+currPos[0] == 0 || currPos[0] == 2) && phi == 0){
      wheel_speed = 7;
    }       
    else{
      count_stop = 0;
      wheel_speed = 10;
    }       
    if(currPos[0] == 1){  
      high(15);
    }
    else{
      low(15);
    }            
    basic_line_follower();
    int prev_speed;
    if(input(0) == 1 && phi == 0){ 
      prev_speed = wheel_speed;
      wheel_speed = 30;   
      servo_angle(leftServo,900 - wheel_speed);
      servo_angle(rightServo,900 +  wheel_speed);
      //display("front\n",-1);
      pause(800);
      // stops the motor out of minor corrections
      if (count_stop == 0){
      servo_angle(leftServo,900);
      servo_angle(rightServo,900);
      pause(6000);
      high(15);
      count_stop++;
      //wheel_speed = prev_speed;
      }      
      if (segment%3 == 1){
       special = 1;
      }      
    }                 
  }
}
