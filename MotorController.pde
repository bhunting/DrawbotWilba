import processing.serial.*;

static final int SM_CMD_MAX_STEPS = 32000; // really 32767

class MotorController
{
    Serial _port;

    boolean _useSmartGeometry = true;
    boolean _dryRun = false;
    boolean _stopped = false;
    boolean _plotMode = false;
    boolean _resumeMode = false;
    int _resumeMode_penState = 0;
    int _resumeMode_commandCount = 0;

    int _penState = 0;
    int _commandCount = 0;

    float _gearCircumference = 90.5;// was 89.5;
    float _gearDiameter = _gearCircumference / PI;

    float _machineAxleWidth = 966.0; // 903.0 + (2.0*31.5);
    float _machineWidth = _machineAxleWidth - _gearDiameter; // 966.0 - 28.4887348
    float _machineHeight = 1200.0;

    float _penOffsetY = 6.718;
    float _penOffsetX = 6.718;

    int _motorStepsPerRev = 3200;
    //int _motorTimePerRevPenUp = 250;//1000; // ms per rev.
    //int _motorTimePerRevPenDown = 500;//1000; // ms per rev.
    float _motorSpeedPenUp = 15.0; // mm/s   (was 5)
    float _motorSpeedPenDown = 15.0; // mm/s  ( was 5)
    float _motorSpeedSetup = 5.0; // mmm/s
    int _motorA_Dir = 1;
    int _motorB_Dir = -1;
    float _setupMoveMotorsStepSize = 16.0; 

    // these are the pen up/pen down positions.
    // NOTE: servo rotation is dependent on brand!
    int _servoPosUp = 16300;
    int _servoPosDown = 10000;
    int _servoRateDown = 600; // was 300
    int _servoRateUp = 600; // was 300

        // in mm
    float _homePosY = 0;
    PVector _homeAB = new PVector(0, 0);

    // in steps, always positive line length
    int _deltaStepsA;
    int _deltaStepsB;
    PVector _currentPos = new PVector(0, 0); // cached because we can't yet calculate from A,B
    PVector _plotPos = new PVector(0, 0);

    // stats
    int _statsPenUpDuration = 0; // time spent with pen up, including pen up/down transitions
    int _statsPenDownDuration = 0; // time spent plotting with pen down
    float _statsPenUpDistance = 0; // distance spent moving with pen up
    float _statsPenDownDistance = 0; // distance spent moving with pen down

    PVector _preDryRun_currentPos;
    int _preDryRun_deltaStepsA;
    int _preDryRun_deltaStepsB;
    int _preDryRun_penState;

    MotorController( PApplet applet, String portName )
    {


        // Can't work out how to get error result from Serial.
        // Only open portName if it is in the list of ports.
        for ( int i=0; i<Serial.list().length; i++ )
        {
            println(Serial.list()[i]);
            if ( Serial.list()[i].equals( portName ) )
            {
                _port = new Serial(applet, portName, 9600);
            }
        }

        setServoSettings();

        // force pen up
        _penState = 0;
        penUp();

        setHome( 250.0 - 21.5 );
    }

    void startDryRun()
    {
        _dryRun = true;

        _preDryRun_currentPos = _currentPos.get();
        _preDryRun_deltaStepsA = _deltaStepsA;
        _preDryRun_deltaStepsB = _deltaStepsB;
        _preDryRun_penState = _penState;

        _statsPenUpDuration = 0;
        _statsPenDownDuration = 0;
        _statsPenUpDistance = 0;
        _statsPenDownDistance = 0;
    }

    void endDryRun()
    {
        _dryRun = false;

        _currentPos = _preDryRun_currentPos.get();
        _plotPos = _preDryRun_currentPos.get();
        _deltaStepsA = _preDryRun_deltaStepsA;
        _deltaStepsB = _preDryRun_deltaStepsB;
        _penState = _preDryRun_penState;
    }

    void startPlot()
    {
        if ( ! _dryRun )
        {
            // query button state to ignore presses before plot starts
            sendCommand( "QB\r" );
        }
        _plotMode = true;
        _stopped = false;
        _commandCount = 0;   
        // _resumeMode might be true or false
    }

    void endPlot()
    {
        _plotMode = false;
        _stopped = false;
        _commandCount = 0;
        _resumeMode = false;
    }

    float XYtoLength( float x, float y, float w, float r )
    {
        //println("XYtoLength("+x+","+y+","+w+","+r+")");
        float d = sqrt( x*x + y*y );
        float m = sqrt( d*d - r*r );

        float a = asin( x / d );
        float b = asin( m / d );
        float c = PI - a - b;
        float n = c * r;

        return m + n;
    }

    PVector XYtoAB( PVector p )
    {
        if ( _useSmartGeometry )
        {
            // Currently using (left motor axle + gear radius, 0 ) as the origin
            // and _machineWidth as distance between gears along axis between motor axles
            PVector axle0 = new PVector( 0 - (_gearDiameter/2.0), 0 );
            PVector axle1 = new PVector( _machineWidth + (_gearDiameter/2.0), 0 );

            PVector p0 = new PVector( p.x - _penOffsetX, p.y - _penOffsetY );
            PVector p1 = new PVector( p.x + _penOffsetX, p.y - _penOffsetY );

            float x0 = p0.x - axle0.x;
            float x1 = axle1.x - p1.x;
            float y = p0.y; // == p1.y
            float w = axle1.x - axle0.x;
            float r = _gearDiameter/2.0;

            float a = XYtoLength( x0, y, w, r );
            float b = XYtoLength( x1, y, w, r );

            return new PVector( a, b );
        }
        else
        {
            // simple eq. for now.
            PVector p0 = new PVector( p.x - _penOffsetX, p.y - _penOffsetY );
            PVector p1 = new PVector( p.x + _penOffsetX, p.y - _penOffsetY );
            float a = dist( 0, 0, p0.x, p0.y );
            float b = dist( _machineWidth, 0, p1.x, p1.y );
            return new PVector( a, b );
        }
    }

    PVector ABtoXY( PVector p )
    {
        float a = p.x;
        float b = p.y;
        //float w = _machineWidth;
        float f = _machineWidth - (2*_penOffsetX);
        //float x = ( w*w - b*b + a*a ) / ( 2*w );
        float x = ( f*f - b*b + a*a ) / ( 2*f );
        float y = sqrt( a*a - x*x );
        return new PVector( x + _penOffsetX, y + _penOffsetY );
    }

    void _delay( int duration )
    {
        try
        {
            Thread.sleep(duration);
        }
        catch (InterruptedException e) {
        }
    }

    String sendCommand( String command )
    {
        if ( _dryRun )
        {
            return null;
        }

        String response = null;
        if ( _port != null )
        {
            //println("<<"+command+">>");
            _port.write(command);

            String s;
            for ( int i=0; i<20; i++ )
            {
                s = _port.readStringUntil('\n');
                if ( s != null )
                {
                    // trim off trailing "\r\n"
                    s = s.substring( 0, s.length()-2 );

                    if ( s.compareTo("OK")==0 )
                    {
                        return response;
                    }
                    else
                    {
                        response = s;
                    }
                }
                _delay(50);
            }
        }

        return response;
    }

    void queryButton()
    {
        if ( _dryRun )
        {
            return;
        }

        String response = sendCommand( "QB\r" );
        if ( response != null && response.compareTo("1")==0 )
        {
            _stopped = true;
            _resumeMode_commandCount = _commandCount;
            _resumeMode_penState = _penState;
            println("stopped after command "+_resumeMode_commandCount+" at "+_currentPos.x+","+_currentPos.y+" pen="+_resumeMode_penState );
        }
    }

    void setServoSettings()
    {
        // NOTE: EBB command doco reads: "SC,4,<servo_min>" and "SC,5,<servo_max>" 
        // BAD NAMING! Really should be called <servo_pos_up> <servo_pos_down>
        // since pen up command moves servo to <servo_min> at <servo_rate_up>
        // pen down command moves servo to <servo_max> at <servo_rate_down>
        sendCommand( "SC,4," + _servoPosUp + "\r");
        sendCommand( "SC,5," + _servoPosDown + "\r");
        // sendCommand( "SC,10," + min(_servoRateUp,_servoRateDown) + "\r");
        sendCommand( "SC,11," + _servoRateUp + "\r");
        sendCommand( "SC,12," + _servoRateDown + "\r");
    }

    PVector getHome()
    {
        return new PVector( _machineWidth/2, _homePosY );
    }

    void setHome( float y )
    {
        _homePosY = y;
        _homeAB = XYtoAB( getHome() );
        _currentPos.set( getHome() );
        _plotPos.set( getHome() );

        println("_homeAB="+_homeAB.x + "," + _homeAB.y );

        _deltaStepsA = 0;
        _deltaStepsB = 0;
    }

    PVector getCurrentAB()
    {
        return new PVector( _homeAB.x + ( _deltaStepsA * ( _gearCircumference / _motorStepsPerRev ) ), 
        _homeAB.y + ( _deltaStepsB * ( _gearCircumference / _motorStepsPerRev ) ), 0 );
    }

    // The calculated position of the pen based on changed line lengths
    // which were rounded to nearest step.
    PVector getCurrentXY()
    {
        if ( _useSmartGeometry )
        {
            return _currentPos;
        }
        else
        {
            return ABtoXY( getCurrentAB() );
        }
    }

    void penUp()
    {
        if ( _stopped || _resumeMode )
        {
            return;
        }

        if ( _penState == 0 )
        {
            int duration = round( (float)(abs(_servoPosUp - _servoPosDown)) / (float)_servoRateUp ) * 24;
            // This doesn't work for large durations.
            //sendCommand("SP,1," + duration + "\r");
            sendCommand("SP,1\r");
            // Do it the EggBot Inkscape plugin way.
            delayMotors( duration );
            _penState = 1;

            _statsPenUpDuration += duration;
        }
    }

    void penDown()
    {
        if ( _stopped || _resumeMode )
        {
            return;
        }

        if ( _penState == 1 )
        {
            int duration = round( (float)(abs(_servoPosUp - _servoPosDown)) / (float)_servoRateDown ) * 24;
            // This doesn't work for large durations.
            //sendCommand("SP,0," + duration + "\r");
            sendCommand("SP,0\r");
            // Do it the EggBot Inkscape plugin way.
            delayMotors( duration );
            _penState = 0;

            _statsPenUpDuration += duration;
        }
    }

    void delayMotors( int duration )
    {
        // Code copied from EggBot Inkscape plugin.
        // I assume "SM" command doesn't like duration more than 750ms
        // when steps are zero.
        while ( duration > 0 )
        {
            int d = min(duration, 750);
            sendCommand("SM,"+d+",0,0\r");
            duration -= d;
        }
    }


    void lineTo( float x, float y )
    {
        lineTo( new PVector( x, y ) );
    }

    void lineTo( PVector pt )
    {
        // inelegant first draft
        // just divide into 1mm segments

        // must copy _plotPos because it gets updated during moveTo()
        PVector startPt = _plotPos.get();
        PVector endPt = pt;

        float segmentLength = 2.0;
        float lineLength = dist(startPt.x, startPt.y, endPt.x, endPt.y );
        int steps = ceil( lineLength / segmentLength ); // round up
        //println( "lineTo(): lineLength=" + lineLength + " steps=" + steps );


        for ( int i=1; i<=steps; i++ )
        {
            float t = (float)i / (float)steps;

            float x = lerp( startPt.x, endPt.x, t );
            float y = lerp( startPt.y, endPt.y, t );

            moveTo( x, y );
        }
    }

    void moveToHome()
    {
        moveTo( getHome() );
    }

    void moveTo( float x, float y )
    {
        moveTo( new PVector( x, y ) );
    }

    void moveTo( PVector pt )
    {
        if ( _stopped )
        {
            return;
        }

        // we are doing the nth command now.
        _commandCount++;
        _plotPos = pt.get();

        if ( _resumeMode && _commandCount < _resumeMode_commandCount )
        {
            //println("resume mode ("+_resumeMode_commandCount+"): skipping command "+_commandCount+" moveTo "+pt.x+","+pt.y);
            return;
        }

        if ( _resumeMode )
        {
            //println("resume mode ("+_resumeMode_commandCount+"): executing command "+_commandCount+" moveTo "+pt.x+","+pt.y);
        }
        else
        {
            //println("executing command "+_commandCount+" moveTo "+pt.x+","+pt.y);
        }

        PVector newAB = XYtoAB( pt );
        PVector currentAB = getCurrentAB();
        PVector currentXY = getCurrentXY(); // NB: this is the REAL position of the pen, different to _plotPos

        int stepsA = round( ( newAB.x - currentAB.x ) / ( _gearCircumference / _motorStepsPerRev ) );
        int stepsB = round( ( newAB.y - currentAB.y ) / ( _gearCircumference / _motorStepsPerRev ) );

        float speed = ( _penState == 0 ) ? _motorSpeedPenDown : _motorSpeedPenUp;
        float distance = dist( currentXY.x, currentXY.y, pt.x, pt.y );
        int duration = round( distance / speed * 1000.0);
        //println("moveTo: _penState="+_penState+" distance="+distance+" speed="+speed+" duration="+duration);
        moveMotors( stepsA, stepsB, duration, /*updateDeltaSteps=*/ true );
        // cache the REAL position of the pen (because we can't calc it from A,B just yet)
        _currentPos = pt.get();

        if ( _penState == 0 )
        {
            _statsPenDownDuration += duration;
            _statsPenDownDistance += distance;
        }
        else
        {
            _statsPenUpDuration += duration;
            _statsPenUpDistance += distance;
        }

        // we resume by re-doing the last move command before stopping, and then resetting pen state
        // to what it was at the time of stopping.
        if ( _resumeMode && _commandCount == _resumeMode_commandCount )
        {
            //println("resume mode ("+_resumeMode_commandCount+"): after command "+_commandCount+" moveTo "+pt.x+","+pt.y+" pen="+_resumeMode_penState);
            _resumeMode = false;
            if ( _resumeMode_penState == 0 )
            {
                //println("pen down to resume");
                penDown();
            }
            // in theory, we're now out of resume mode state and could just
            // set _resumeMode to false.
        }

        if ( _plotMode )
        {   
            queryButton();
        }
    }

    void setSetupMoveMotorStepSize( float setupMoveMotorsStepSize )
    {
        _setupMoveMotorsStepSize = setupMoveMotorsStepSize;
    }

    void setupMoveMotors( float directionA, float directionB )
    {
        // HACK: args in steps, convert to distance
        float distanceA = directionA * _setupMoveMotorsStepSize * ( _gearCircumference / _motorStepsPerRev );
        float distanceB = directionB * _setupMoveMotorsStepSize * ( _gearCircumference / _motorStepsPerRev );
        int stepsA = round( distanceA / ( _gearCircumference / _motorStepsPerRev ) );
        int stepsB = round( distanceB / ( _gearCircumference / _motorStepsPerRev ) );

        float speed = _motorSpeedSetup;
        float distance = max( abs(distanceA), abs(distanceB) );
        int duration = round( distance / speed * 1000.0);

        moveMotors( stepsA, stepsB, duration, /*updateDeltaSteps=*/ false );
    }

    // this handles steps greater than the maximum steps per motor move command.
    // also will do nothing if both step values are zero
    void moveMotors( int stepsA, int stepsB, int duration, boolean updateDeltaSteps )
    {
        int totalStepsA = stepsA;
        int totalStepsB = stepsB;
        int totalDuration = duration;
        while ( (totalStepsA != 0 || totalStepsB != 0) )
        {
            stepsA = totalStepsA;
            stepsB = totalStepsB;
            duration = totalDuration;

            float f = 1.0;
            if ( abs(stepsA) > SM_CMD_MAX_STEPS || abs(stepsB) > SM_CMD_MAX_STEPS )
            {
                f = (float)SM_CMD_MAX_STEPS / abs((float)max(abs(stepsA), abs(stepsB)));
            }

            // avoid all possibility of float math causing value to exceed actual maximum
            stepsA = constrain( round(f * stepsA), -SM_CMD_MAX_STEPS, SM_CMD_MAX_STEPS );
            stepsB = constrain( round(f * stepsB), -SM_CMD_MAX_STEPS, SM_CMD_MAX_STEPS );
            duration = constrain( round(f * duration), 1, 65535 );

            sendCommand("SM," + duration + "," + _motorA_Dir*stepsA + "," + _motorB_Dir*stepsB + "\r" );

            //float rpsA = (float)stepsA / ((float)duration / 1000.0) / (float)_motorStepsPerRev;
            //float rpsB = (float)stepsB / ((float)duration / 1000.0) / (float)_motorStepsPerRev;
            //PVector currentXY = getCurrentXY();
            //PVector newXY = ABtoXY( new PVector( getCurrentAB().x + (float)stepsA/(float)_motorStepsPerRev*(float)_gearCircumference,
            //                                    getCurrentAB().y + (float)stepsB/(float)_motorStepsPerRev*(float)_gearCircumference ) );
            //float speed = dist(currentXY.x, currentXY.y, newXY.x, newXY.y) / ((float)duration/1000.0);
            //println("speedA="+ rpsA*_gearCircumference +"mm/s speedB=" + rpsB*_gearCircumference + "mm/s speed="+speed+"mm/s");

            // totalStepsA,totalStepsB will approach zero.
            totalStepsA -= stepsA;
            totalStepsB -= stepsB;
            totalDuration -= duration;

            // track total change in motor position.
            // this is the source of current actual position of pen
            if ( updateDeltaSteps )
            {
                _deltaStepsA += stepsA;
                _deltaStepsB += stepsB;
            }
        }
    }


    void circle( PVector pt, float radius )
    {
        float segmentLength = 1.0;
        float circumference = PI * 2.0 * radius;
        int steps = ceil( circumference / segmentLength ); // round up
        if ( steps < 8 )
        {
            steps = 8;
        }

        if ( radius <= 0 )
        {
            moveTo( pt.x, pt.y );
            penDown();
        }
        else
        {
            for ( int i=0; i<=steps; i++ )
            {
                float t = (float)i / (float)steps * 2.0 * PI;

                float x = pt.x + cos( t ) * radius;
                float y = pt.y - sin( t ) * radius;

                moveTo( x, y );
                penDown();
            }
        }
    }

    void fillCircle( PVector pt, float radius, float penWidth )
    {
        float r = radius;
        while ( r >= 0 )
        {
            circle( pt, r );
            r -= penWidth;
        }
        moveTo( pt.x, pt.y );
    }
}

