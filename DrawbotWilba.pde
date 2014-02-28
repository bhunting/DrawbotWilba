/*************************************************
 * Drawbot code to drive a polar graphic plotter.
 * MotorController is presently coded to support the
 * EiBotBoard (EggBotBoard) from Brian Schmalz 
 * http://www.schmalzhaus.com/EBB/
 *
 * Original code is from Wilba6582 (Jason Williams) and is
 * released with the copyright
 * "I hereby release that source code package as Creative Commons blah blah free for anyone to use for any purposes."
 *
 * Modifications by Brad Hunting released under a Creative Commons Attribution 4.0 International License.
 *
 * Released files include DrawbotWilba.pde, MotorController.pde, ShapeManager.pde
 *
 ************************************************/

import java.awt.event.*;
import controlP5.*;

ControlP5 cp5;
Textarea debugTextarea;
Println debugConsole;

PVector _screenTranslate = new PVector(0, 0);
float _screenScale = 1;

PVector _mouse = new PVector(0, 0);
PVector _mousePress = null;

PVector _lineStart = new PVector(0, 0);
PVector _lineEnd = new PVector(0, 0);

int _guiHeight = 200;
boolean _guiInitialized = false;

MotorController _motorController;
ShapeManager _shapeManager = new ShapeManager();
RPoint[][] _shapePoints;

PImage _image;
float _imageScale = 1; // pixels per mm ??
float _pageOffsetY = 0;
PVector _pageSize = new PVector( 210, 297 ); // A4 portrait
PVector _shapeOffset = new PVector(0, 0);

//-----------------------------------------------------------------------------
void setup()
{
  size(900, 1000 );

  // VERY IMPORTANT: Allways initialize the library before using it
  RG.init(this);

  // Create MotorController before GUI
  // then controls can be init with values
  _motorController = new MotorController(this);

  cp5 = new ControlP5(this);
  cp5.enableShortcuts();
  setupGUI();

  _motorController.init("COM20");
  setupGamepad();

  addMouseWheelListener(new MouseWheelListener() 
                        { public void mouseWheelMoved(MouseWheelEvent mwe) 
                             { mouseWheel(mwe.getWheelRotation()); } } ); 

  _screenTranslate.set( 40, 40, 0 );
  _screenScale = 0.91;

  _lineStart = _motorController.getCurrentXY().get();
  _lineEnd = _motorController.getCurrentXY().get();

  _pageOffsetY = 0;

  //_shapeManager.loadShape("test1.svg");
  //_shapeOffset.set( 30, 50, 0 );

  _image = loadImage( "freckles.jpg" );
  _imageScale = 0.195;
  //_imageScale = 0.5;
  _shapeManager.rasterizeImage( _image, _imageScale );
  _shapeOffset.set( 5, 50, 0 );
}

//-----------------------------------------------------------------------------
PVector screenToModel( PVector p )
{
  float x = p.x;
  float y = p.y;
  x -= _screenTranslate.x;
  y -= _screenTranslate.y;
  x *= 1.0/_screenScale;
  y *= 1.0/_screenScale;
  return new PVector(x, y);
}

//-----------------------------------------------------------------------------
PVector modelToScreen( PVector p )
{
  float x = p.x;
  float y = p.y;
  x *= _screenScale;
  y *= _screenScale;
  x += _screenTranslate.x;
  y += _screenTranslate.y;
  return new PVector(x, y);
}

//-----------------------------------------------------------------------------
PVector getPageOrigin()
{
  PVector home = _motorController.getHome();
  return new PVector( home.x - (_pageSize.x/2), home.y + _pageOffsetY );
}

//-----------------------------------------------------------------------------
PVector getShapeOrigin()
{
  PVector pageOrigin = getPageOrigin();
  return new PVector( pageOrigin.x + _shapeOffset.x, pageOrigin.y + _shapeOffset.y );
}

//-----------------------------------------------------------------------------
void draw()
{
  background(192);

  pushMatrix();
  {
    translate(_screenTranslate.x, _screenTranslate.y);
    scale(_screenScale);

    fill(168);
    noStroke();
    rect( 0, 0, _motorController._machineWidth, _motorController._machineHeight );


    pushMatrix();
    {
      PVector pageOrigin = getPageOrigin();
      translate(pageOrigin.x, pageOrigin.y);

      fill(64);
      noStroke();
      rect( 1, 1, _pageSize.x, _pageSize.y );
      fill(255);
      strokeWeight(1/_screenScale);
      stroke(0);
      rect( 0, 0, _pageSize.x, _pageSize.y );
    }
    popMatrix();

    pushMatrix();
    {
      if ( _image != null )
      {
        PVector shapeOrigin = getShapeOrigin();
        translate(shapeOrigin.x, shapeOrigin.y);
        scale( _imageScale );
        tint(255, 255, 255, 30);
        image( _image, 0, 0 );
        noTint();
      }
    }
    popMatrix();

    PVector home = _motorController.getHome();
    noFill();
    strokeWeight(1/_screenScale);
    stroke(0);
    float n = 5;
    line( home.x-n, home.y-n, home.x+n, home.y+n );
    line( home.x+n, home.y-n, home.x-n, home.y+n );

    PVector motorA = new PVector( 0, 0 );
    PVector motorB = new PVector( _motorController._machineWidth, 0 );
    float gearRadius = _motorController._gearDiameter/2;

    strokeWeight(1/_screenScale);
    noFill();

    stroke(#0000FF);
    ellipse( motorA.x - gearRadius, motorA.y, 2*gearRadius, 2*gearRadius );
    stroke(#00FF00);
    ellipse( motorB.x + gearRadius, motorB.y, 2*gearRadius, 2*gearRadius );

    stroke(#0000FF);
    line( motorA.x, motorA.y, _lineStart.x, _lineStart.y );
    line( motorA.x, motorA.y, _lineEnd.x, _lineEnd.y );
    float r;
    r = dist( motorA.x, motorA.y, _lineStart.x, _lineStart.y );
    //ellipse( motorA.x, motorA.y, 2*r, 2*r );
    arc( motorA.x, motorA.y, 2*r, 2*r, 0, PI/2 );
    r = dist( motorA.x, motorA.y, _lineEnd.x, _lineEnd.y );
    //ellipse( motorA.x, motorA.y, 2*r, 2*r );
    arc( motorA.x, motorA.y, 2*r, 2*r, 0, PI/2 );

    stroke(#00FF00);
    line( motorB.x, motorB.y, _lineStart.x, _lineStart.y );
    line( motorB.x, motorB.y, _lineEnd.x, _lineEnd.y );
    r = dist( motorB.x, motorB.y, _lineStart.x, _lineStart.y );
    //ellipse( motorB.x, motorB.y, 2*r, 2*r );
    arc(  motorB.x, motorB.y, 2*r, 2*r, PI/2, PI );
    r = dist( motorB.x, motorB.y, _lineEnd.x, _lineEnd.y );
    //ellipse( motorB.x, motorB.y, 2*r, 2*r );
    arc(  motorB.x, motorB.y, 2*r, 2*r, PI/2, PI );

    stroke(#FF0000);
    line( _lineStart.x, _lineStart.y, _lineEnd.x, _lineEnd.y );

    /*
        stroke(#FF00FF);
     
     PVector motorStart = _motorController.XYtoAB( _lineStart );
     PVector motorEnd = _motorController.XYtoAB( _lineEnd ); 
     
     noFill();
     beginShape();
     {
     for ( int i=0; i<=10; i++ )
     {
     float t = (float)i / 10.0;
     
     float mx = lerp( motorStart.x, motorEnd.x, t );
     float my = lerp( motorStart.y, motorEnd.y, t );
     
     PVector p = new PVector( mx, my );
     p = _motorController.ABtoXY( p );
     vertex( p.x, p.y );
     }
     }
     endShape();
     */
  }
  popMatrix();


  pushMatrix();
  {
    translate(_screenTranslate.x, _screenTranslate.y);
    scale(_screenScale);

    RG.ignoreStyles();

    noFill();
    strokeWeight(1/_screenScale);
    stroke(0);

    if ( _shapeManager.getPlotShape() != null )
    {
      pushMatrix();
      PVector shapeOrigin = getShapeOrigin();
      translate(shapeOrigin.x, shapeOrigin.y);
      _shapeManager.getPlotShape().draw();
      popMatrix();

      if ( _shapePoints == null )
      {
        _shapePoints = _shapeManager.getPointsInPaths( _shapeManager.getPlotShape(), shapeOrigin );
      }
      //drawShapePoints(_shapePoints);
    }
  }
  popMatrix();

  // Draw background for GUI area
  fill(64);
  noStroke();
  rect( 0, height-_guiHeight, width, _guiHeight );
}

//-----------------------------------------------------------------------------
void drawShapePoints( RPoint[][] shapePoints )
{
  if ( shapePoints == null )
  {
    return;
  }
  RPoint lastPoint = null;
  for ( int n=0; n<shapePoints.length; n++ )
  {
    RPoint[] points = shapePoints[n];


    // If there are any points
    if (points != null)
    {
      if ( lastPoint != null )
      {
        noFill();
        stroke(192, 192, 255);
        strokeWeight(1/_screenScale);
        if ( false )/*drawPenUpMoves*/
          line(lastPoint.x, lastPoint.y, points[0].x, points[0].y);
      }

      noFill();
      stroke(0, 0, 255);
      strokeWeight(1/_screenScale);
      beginShape();
      for (int i=0; i<points.length; i++)
      {
        if ( false )/*drawPenDownMoves*/
          vertex(points[i].x, points[i].y);
      }
      endShape();


      fill(0);
      stroke(255, 0, 0);
      strokeWeight(3/_screenScale);
      for (int i=0; i<points.length; i++)
      {
        if ( true )/*drawNodes*/
          point(points[i].x, points[i].y);
      }

      lastPoint = points[points.length-1];
    }
  }
}

//-----------------------------------------------------------------------------
void mousePressed()
{
  if ( mouseY > height-_guiHeight )
  {
    // ignore mouse presses in GUI area
    return;
  }

  _mouse.set( mouseX, mouseY, 0);
  _mousePress = new PVector(mouseX, mouseY);

  if ( mouseButton==LEFT)
  {
    _lineStart = screenToModel(_mouse);
    _lineEnd = _lineStart;
  }

  if ( mouseButton==RIGHT )
  {
  }
}

//-----------------------------------------------------------------------------
void mouseReleased()
{
  if ( mouseButton==LEFT && _mousePress != null )
  {
    _motorController.penUp();
    _motorController.moveTo( _lineStart );
    if ( _lineStart.x != _lineEnd.x || _lineStart.y != _lineEnd.y )
    {
      _motorController.penDown();
      _motorController.lineTo( _lineEnd );
      _motorController.penUp();


      /*
            float x = _lineStart.x;
       float y = _lineStart.y;
       float r = 0.1;
       for ( int i=1; i<=10; i++ )
       {
       _motorController.fillCircle( new PVector(x,y), r, 0.4 );
       _motorController.penUp();
       x += 3*r+2;
       r += 0.1;
       }
       */
    }
  }
  _mousePress = null;
}

//-----------------------------------------------------------------------------
void mouseDragged()
{
  if ( _mousePress == null )
  {
    // ignore drags starting in the GUI area
    return;
  }

  PVector newMouse = new PVector( mouseX, mouseY );
  if ( mouseButton == LEFT && mouseY <= height-_guiHeight )
  {
    _lineEnd = screenToModel(newMouse);
  }

  if ( mouseButton==CENTER )
  {
    _screenTranslate.set( _screenTranslate.x + (newMouse.x - _mouse.x), _screenTranslate.y + (newMouse.y - _mouse.y), 0 );
  }

  _mouse = newMouse;
}

void mouseWheel(int delta)
{
  PVector oldMouse = screenToModel( new PVector(mouseX, mouseY) );
  _screenScale *= 1 + (float)delta/10.0;
  oldMouse = modelToScreen( oldMouse );
  _screenTranslate.set( _screenTranslate.x + (mouseX - oldMouse.x ), _screenTranslate.y + ( mouseY - oldMouse.y ), 0 );
}


///////////////////////////////////
// Setup of GUI controls
///////////////////////////////////
//-----------------------------------------------------------------------------
void setupGUI()
{
  PVector guiTopLeft = new PVector( 0, height - _guiHeight );
  int x, y;


  cp5.getWindow().setPositionOfTabs( (int)guiTopLeft.x, (int)guiTopLeft.y );

  cp5.getTab("default")
    .activateEvent(true)
      .setLabel("main")
        .setId(1)
          ;

  cp5.addTab("motor_setup")
    .activateEvent(true)
      .setId(2)
        ;

  cp5.addTab("debug")
    .activateEvent(true)
      .setId(3)
        ;

  y = (int)guiTopLeft.y + 20;
  x = (int)guiTopLeft.x + 20;

  cp5.addButton("btnPlot")
    .setPosition(x, y)
      .setSize(80, 20)
        .setCaptionLabel("Plot")
          ;   

  cp5.addButton("btnPlotTime")
    .setPosition(x, y+20)
      .setSize(80, 20)
        .setCaptionLabel("Plot Time")
          ;   
  cp5.addButton("btnResumePlot")
    .setPosition(x, y+40)
      .setSize(80, 20)
        .setCaptionLabel("Resume Plot")
          ;                          
  cp5.addButton("btnPlotStipple")
    .setPosition(x, y+60)
      .setSize(80, 20)
        .setCaptionLabel("Plot Stipple")
          ;                          

  cp5.addButton("btnPlotStippleTime")
    .setPosition(x, y+80)
      .setSize(80, 20)
        .setCaptionLabel("Plot Stipple Time")
          ;                           

  cp5.addButton("btnMoveToHome")
    .setPosition(x, y)
      .setSize(60, 20)
        .setCaptionLabel("Move To Home")
          .moveTo("motor_setup")
            ;

  cp5.addButton("btnTestPattern")
    .setPosition(x, y+40)
      .setSize(60, 20)
        .setCaptionLabel("Test Pattern")
          .moveTo("motor_setup")
            ;    
  cp5.addButton("btnPenUp")
    .setPosition(x, y+60)
      .setSize(60, 20)
        .setCaptionLabel("Pen Up")
          .moveTo("motor_setup")
            ;   
  cp5.addButton("btnPenDown")
    .setPosition(x, y+80)
      .setSize(60, 20)
        .setCaptionLabel("Pen Down")
          .moveTo("motor_setup")
            ;   
  cp5.addButton("btnVersion")
    .setPosition(x, y+100)
      .setSize(60, 20)
        .setCaptionLabel("Version")
          .moveTo("motor_setup")
            ;   

  cp5.addButton("btnQueryButton")
    .setPosition(x, y+120)
      .setSize(60, 20)
        .setCaptionLabel("QueryBtn")
          .moveTo("motor_setup")
            ;   

   cp5.addNumberbox("ctrlMotorSpeedPenUp",   _motorController._motorSpeedPenUp,   x+120, y+0*20, 100, 14).setId(3).setRange(0,200).setMultiplier(1).setDirection(Controller.HORIZONTAL).moveTo("motor_setup");
   cp5.addNumberbox("ctrlMotorSpeedPenDown", _motorController._motorSpeedPenDown, x+120, y+2*20, 100, 14).setId(4).setRange(0,200).setMultiplier(1).setDirection(Controller.HORIZONTAL).moveTo("motor_setup");  
   cp5.addNumberbox("ctrlServoPosUp",        _motorController._servoPosUp,        x+120, y+4*20, 100, 14).setId(5).setRange(0,200).setMultiplier(1).setDirection(Controller.HORIZONTAL).moveTo("motor_setup");         
   cp5.addNumberbox("ctrlServoPosDown",      _motorController._servoPosDown,      x+300, y+0*20, 100, 14).setId(6).setRange(0,200).setMultiplier(1).setDirection(Controller.HORIZONTAL).moveTo("motor_setup");        
   cp5.addNumberbox("ctrlServoRateDown",     _motorController._servoRateDown,     x+300, y+2*20, 100, 14).setId(7).setRange(0,200).setMultiplier(1).setDirection(Controller.HORIZONTAL).moveTo("motor_setup");         
   cp5.addNumberbox("ctrlServoRateUp",       _motorController._servoRateUp,       x+300, y+4*20, 100, 14).setId(8).setRange(0,200).setMultiplier(1).setDirection(Controller.HORIZONTAL).moveTo("motor_setup");         

  x = (int)guiTopLeft.x + 600;
  int buttonSize = 40;

  cp5.addButton("btnMoveUpLeft")
    .setPosition(x+0*buttonSize, y+0*buttonSize)
      .setSize(buttonSize-1, buttonSize-1)
        .setCaptionLabel("\\")
          .moveTo("motor_setup")
            ;

  cp5.addButton("btnMoveUp")
    .setPosition(x+1*buttonSize, y+0*buttonSize)
      .setSize(buttonSize-1, buttonSize-1)
        .setCaptionLabel("/\\")
          .moveTo("motor_setup");
  ;
  cp5.addButton("btnMoveUpRight")
    .setPosition(x+2*buttonSize, y+0*buttonSize)
      .setSize(buttonSize-1, buttonSize-1)
        .setCaptionLabel("/")
          .moveTo("motor_setup")
            ;

  cp5.addButton("btnMoveLeft")
    .setPosition(x+0*buttonSize, y+1*buttonSize)
      .setSize(buttonSize-1, buttonSize-1)
        .setCaptionLabel("<")
          .moveTo("motor_setup")
            ;
  cp5.addButton("btnMove")
    .setPosition(x+1*buttonSize, y+1*buttonSize)
      .setSize(buttonSize-1, buttonSize-1)
        .moveTo("motor_setup")
          ;
  cp5.addButton("btnMoveRight")
    .setPosition(x+2*buttonSize, y+1*buttonSize)
      .setSize(buttonSize-1, buttonSize-1)
        .setCaptionLabel(">")
          .moveTo("motor_setup")
            ; 

  cp5.addButton("btnMoveDownLeft")
    .setPosition(x+0*buttonSize, y+2*buttonSize)
      .setSize(buttonSize-1, buttonSize-1)
        .setCaptionLabel("/")
          .moveTo("motor_setup")
            ;

  cp5.addButton("btnMoveDown")
    .setPosition(x+1*buttonSize, y+2*buttonSize)
      .setSize(buttonSize-1, buttonSize-1)
        .setCaptionLabel("\\/")
          .moveTo("motor_setup")
            ;

  cp5.addButton("btnMoveDownRight")
    .setPosition(x+2*buttonSize, y+2*buttonSize)
      .setSize(buttonSize-1, buttonSize-1)
        .setCaptionLabel("\\")
          .moveTo("motor_setup")
            ;
            
  debugTextarea = cp5.addTextarea("debugtext")
                  .setPosition(100, 100)
                  .setSize(200, 200)
                  .setFont(createFont("", 10))
                  .setLineHeight(14)
                  .setColor(color(200))
                  .setColorBackground(color(0, 100))
                  .setColorForeground(color(255, 100))
                  .moveTo("motor_setup");

  //debugConsole = cp5.addConsole(debugTextarea);//
              
}

//-----------------------------------------------------------------------------
public void controlEvent(ControlEvent theEvent) 
{
  //println("got a control event from controller with id "+theEvent.getId());
   switch(theEvent.getId()) 
   {
     case(1): // numberboxA is registered with id 1
       //myColorRect = (int)(theEvent.getController().getValue());
       //println("EVENT 1");
       break;
     case(2):  // numberboxB is registered with id 2
       //myColorBackground = (int)(theEvent.getController().getValue());
       //println("EVENT 2");
       break;
     default:
       //println("EVENT " + theEvent.getId());
       break;
   }
}

//-----------------------------------------------------------------------------
public void ctrlMachineWidth( float theValue ) 
{
  _motorController._machineWidth = theValue;
}


//-----------------------------------------------------------------------------
public void btnPlot(int theValue) 
{
  _motorController.startPlot();

  RPoint[][] shapePoints = _shapeManager.getPointsInPaths( _shapeManager.getPlotShape(), getShapeOrigin() );

  if ( shapePoints == null )
  {
    return;
  }

  for ( int n=0; n<shapePoints.length; n++ )
  {
    RPoint[] points = shapePoints[n];

    // If there are any points
    if (points != null)
    {
      for (int i=0; i<points.length; i++)
      {
        if ( i == 0 )
        {
          _motorController.moveTo( points[i].x, points[i].y );
          _motorController.penDown();
        }
        else
        {
          _motorController.lineTo( points[i].x, points[i].y );
        }
      }
      _motorController.penUp();
    }
  }

  if ( _motorController._stopped )
  {
    println("Plot cancelled by user");
  }

  _motorController.endPlot();
  _motorController.penUp();
  _motorController.moveToHome();
}

//-----------------------------------------------------------------------------
public void btnResumePlot(int theValue) 
{
  _motorController._resumeMode = true;
  btnPlot(0);
  _motorController._resumeMode = false;
}

//-----------------------------------------------------------------------------
public void btnPlotTime(int theValue) 
{
  _motorController.startDryRun();
  btnPlot(0);
  _motorController.endDryRun();
  printDryRunStats();
}


//-----------------------------------------------------------------------------
public void btnPlotStipple( int theValue ) 
{

  _motorController.startPlot();

  RShape sourceShape = _shapeManager.getPlotShape();

  for ( int i=0; i<sourceShape.countChildren(); i++ )
  {
    RShape shp = sourceShape.children[i];

    float w = shp.getWidth();
    float h = shp.getHeight();
    RPoint c = shp.getCenter();
    float r = w / 2.0;
    //println("w="+w);

    _motorController.fillCircle( new PVector( c.x, c.y), r, 0.4 );
    _motorController.penUp();
  }

  if ( _motorController._stopped )
  {
    println("Plot cancelled by user");
  }
  _motorController.endPlot();
  _motorController.penUp();
  _motorController.moveToHome();
}

//-----------------------------------------------------------------------------
public void btnPlotStippleTime(int theValue) 
{
  _motorController.startDryRun();
  btnPlotStipple(0);
  _motorController.endDryRun();
  printDryRunStats();
}


//-----------------------------------------------------------------------------
public void btnMoveToHome(int theValue) 
{
  _motorController.moveToHome();
}

//-----------------------------------------------------------------------------
public void btnTestPattern0(int theValue) 
{


  float homeX = _motorController.getHome().x;
  float homeY = _motorController.getHome().y;

  float x = homeX + 0;
  float y = homeY + 270;

  _motorController.moveTo( x-20, y );
  _motorController.penDown();        
  _motorController.lineTo( x+20, y );
  _motorController.penUp();
  _motorController.moveToHome();
}

//-----------------------------------------------------------------------------
public void btnTestPattern(int theValue) 
{


  float homeX = _motorController.getHome().x;
  float homeY = _motorController.getHome().y;
  homeY += 10;

  float x, y;

  int i=0;

  float gridSize = 10;
  float cellsX = 14;
  float cellsY = 20;

  float dx = cellsX / 2 * 10;
  for ( i=0; i<=cellsY; i++ )
  {
    y = homeY + i * gridSize;

    _motorController.moveTo( homeX-dx, y );
    _motorController.penDown();        
    _motorController.lineTo( homeX+dx, y );
    _motorController.penUp();
    dx = -dx;
  }

  float dy = cellsY / 2 * 10;
  float y2 = homeY + dy;
  float x2 = homeX - abs(dx);
  for ( i=0; i<=cellsX; i++ )
  {
    x = x2 + i * gridSize;
    _motorController.moveTo( x, y2-dy );
    _motorController.penDown();        
    _motorController.lineTo( x, y2+dy );
    _motorController.penUp();
    dy = -dy;
  }
  _motorController.moveToHome();
}

//-----------------------------------------------------------------------------
public void btnTestPattern2(int theValue) 
{


  float x = _motorController.getHome().x+00;
  float y = _motorController.getHome().y+150;

  float d = 60;
  _motorController.moveTo( x-d, y-d );
  _motorController.penDown();        
  _motorController.lineTo( x+d, y-d );
  _motorController.lineTo( x+d, y+d );
  _motorController.lineTo( x-d, y+d );
  _motorController.lineTo( x-d, y-d );
  _motorController.lineTo( x+d, y+d );
  _motorController.penUp();
  _motorController.moveTo( x+d, y-d );
  _motorController.penDown();
  _motorController.lineTo( x-d, y+d );       
  _motorController.penUp();
  _motorController.moveToHome();
}

//-----------------------------------------------------------------------------
public void btnPenUp(int theValue) 
{     
  _motorController.penUp();
}

//-----------------------------------------------------------------------------
public void btnPenDown(int theValue) 
{     
  _motorController.penDown();
}

//-----------------------------------------------------------------------------
public void btnMoveUpLeft(int theValue) 
{
  println("btnMoveUpLeft");
  _motorController.setupMoveMotors( -1, 0 );
}

//-----------------------------------------------------------------------------
public void btnMoveUp(int theValue) 
{
  _motorController.setupMoveMotors( -1, -1 );
}

//-----------------------------------------------------------------------------
public void btnMoveUpRight(int theValue) 
{
  _motorController.setupMoveMotors( 0, -1 );
}

//-----------------------------------------------------------------------------
public void btnMoveLeft(int theValue) 
{
  _motorController.setupMoveMotors( -1, 1 );
}

//-----------------------------------------------------------------------------
public void btnMoveRight(int theValue) 
{
  _motorController.setupMoveMotors( 1, -1 );
}

//-----------------------------------------------------------------------------
public void btnMoveDownLeft(int theValue) 
{
  _motorController.setupMoveMotors( 0, 1 );
}

//-----------------------------------------------------------------------------
public void btnMoveDown(int theValue) 
{
  _motorController.setupMoveMotors( 1, 1 );
}

//-----------------------------------------------------------------------------
public void btnMoveDownRight(int theValue) 
{
  _motorController.setupMoveMotors( 1, 0 );
}

//-----------------------------------------------------------------------------
public void btnVersion(int theValue) 
{
   debugTextarea.show();
  _motorController.sendCommand("V\r");
}

//-----------------------------------------------------------------------------
public void btnQueryButton(int theValue) 
{
   debugTextarea.hide();
  _motorController.sendCommand("QB\r");
}


//-----------------------------------------------------------------------------
void printDryRunStats()
{
  println( "pen up distance = " + _motorController._statsPenUpDistance + " mm");
  println( "pen down distance = " + _motorController._statsPenDownDistance + " mm");    
  println( "pen up duration = " + _motorController._statsPenUpDuration / 60000.0 + " minutes");
  println( "pen down duration = " + _motorController._statsPenDownDuration / 60000.0 + " minutes");
  println( "total duration = " + ( _motorController._statsPenUpDuration + _motorController._statsPenDownDuration ) / 60000.0 + " minutes");
}


//-----------------------------------------------------------------------------
void setupGamepad()
{
  /*
    joypad = controllIO.getDevice("SmartJoy PLUS Adapter");
   joypad.printButtons();
   
   joypad.plug(this, "onGamepadL2ButtonPress", ControllIO.ON_PRESS, 4);
   joypad.plug(this, "onGamepadL2ButtonRelease", ControllIO.ON_RELEASE, 4);
   joypad.plug(this, "onGamepadL1ButtonPress", ControllIO.ON_PRESS, 6);
   joypad.plug(this, "onGamepadL1ButtonRelease", ControllIO.ON_RELEASE, 6);
   //joypad.plug(this, "onGamepadDpadMovement", ControllIO.WHILE_PRESS, 12);
   //joypad.plug(this, "onGamepadDpadPress", ControllIO.ON_PRESS, 12);
   joypad.plug(this, "onGamepadDpadRelease", ControllIO.ON_RELEASE, 12);
   */
}     

//-----------------------------------------------------------------------------
void onGamepadL1ButtonPress() 
{
  _motorController.setSetupMoveMotorStepSize( 3200.0 );
}

//-----------------------------------------------------------------------------
void onGamepadL1ButtonRelease() 
{
  _motorController.setSetupMoveMotorStepSize( 16.0 );
}

//-----------------------------------------------------------------------------
void onGamepadL2ButtonPress() 
{
  _motorController.penDown();
}

//-----------------------------------------------------------------------------
void onGamepadL2ButtonRelease() 
{
  _motorController.penUp();
}

//-----------------------------------------------------------------------------
void onGamepadDpadPress(final float x, final float y) 
{
}

//-----------------------------------------------------------------------------
void onGamepadDpadRelease(final float x, final float y) 
{
  onGamepadDpadMovement( x, y );
}

//-----------------------------------------------------------------------------
void onGamepadDpadMovement(final float x, final float y) 
{
  /*
    if ( x<0 && y<0 ) _motorController.setupMoveMotors( -1, 0 );
   else if ( x==0 && y<0 ) _motorController.setupMoveMotors( -1, -1 );
   else if ( x>0 && y<0 ) _motorController.setupMoveMotors( 0, -1 );
   
   else if ( x<0 && y==0 ) _motorController.setupMoveMotors( -1, 1 );
   else if ( x>0 && y==0 ) _motorController.setupMoveMotors( 1, -1 );
   
   else if ( x<0 && y>0 ) _motorController.setupMoveMotors( 0, 1 );
   else if ( x==0 && y>0 ) _motorController.setupMoveMotors( 1, 1 );
   else if ( x>0 && y>0 ) _motorController.setupMoveMotors( 1, 0 );
   */
}


