import processing.serial.*;


Serial myPort; // declare port object

int Setpoint = 15, Input = 0, Output = 0;

int [] Inputs = new int[240];
int [] Outputs = new int[120];

void add_sample(int sample, int [] arr, int sz){
  for(int i = 0; i < sz - 1; ++i){
    arr[i] = arr[i+1];
  }
  arr[sz-1] = sample;
}

public static int tryParse(String text) {
  try {
    return Integer.parseInt(text);
  } catch (NumberFormatException e) {
    return -1;
  }
}

void draw_graph(int [] arr, int ymin, int ymax, int samples, int r, int g, int b, int yoffset, int sz){
  double yScale = height/((double)(ymax-ymin));
  double xIncrement = width/(double)samples;
  
  stroke(r,g,b);
  fill(r,g,b);
  
  rectMode(CENTER);
  
  double xpos = 0.0;
  for(int i = 0; i < samples; ++i){
    rect((float)xpos,(float)(height - (double)arr[i]*yScale),(float)sz,(float)sz);
    xpos += xIncrement;
  }
}

void draw_h_line(int pos, int min, int max, int r, int g, int b, String label, int yoffset){
  double yScale = height/((double)(max-min));
  pos = (int)((double)pos*yScale);
  strokeWeight(5);
  stroke(r,g,b);
  line(0.f,(float)(yoffset-(float)pos),(float)width,(float)(yoffset-(float)pos));
  textSize(30);
  fill(r,g,b);
  text(label,12,(float)(yoffset-(float)pos)+30);
}

void setup(){
  size(800,600,P2D);
  frameRate(60);
  
  myPort = new Serial(this, Serial.list()[0], 9600); // initialize port at 9600 baud, will throw an error if no port detected
  printArray(Serial.list());
  delay(4000);
  myPort.bufferUntil('\n');
}

void draw(){
  background(0xFF);
  
  add_sample(Input, Inputs, 240);
  
  draw_graph(Inputs, 0, 27, 240, 0, 0, 255, height, 5);
  draw_h_line(Setpoint,0, 27,255,0,0, "setpoint: " + Setpoint + "cm", height);
}


void serialEvent(Serial b) {
  String inString = myPort.readString();
  if(inString != null){
    int pos = inString.indexOf(',');
    
    if(pos == -1) return;
    String result = inString.substring(0, pos);
    int temp = tryParse(result);
    if(temp == -1) return;
    
    Setpoint = temp;
    println(result);
    result = inString.substring(pos+1);
    
    
    pos = result.indexOf(',');
    if(pos == -1) return;
    
    result = result.substring(0, pos);
    temp = tryParse(result);
    //println(temp);
    if(temp == -1) return;
    
    Input = temp;
  }
}
