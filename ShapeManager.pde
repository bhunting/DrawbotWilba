
import geomerative.*;

static final float PIXELS_TO_MM = 1.0 / ( 90.0 / 25.4 );

//-----------------------------------------------------------------------------
class ShapeManager
{
  RShape _loadedShape;
  RShape _plotShape;

  RShape getLoadedShape()
  {
    return _loadedShape;
  }

  RShape getPlotShape()
  {
    return _plotShape;
  }

  boolean loadShape( String filename )
  {
    RShape shape = RG.loadShape(filename);
    if ( shape != null )
    {
      _loadedShape = flattenShape( shape );
      _loadedShape.scale( PIXELS_TO_MM );

      // nothing to do here (yet).
      // TODO things like hatching/stippling of loaded shape
      // into the plot shape.           
      _plotShape = new RShape( _loadedShape );
      return true;
    }
    else
    {
      _loadedShape = null;
      _plotShape = null;
      return false;
    }
  }


//-----------------------------------------------------------------------------
  float getPixelBrightness( PImage img, float imgScale, float x, float y )
  {
    // pre-scale co-ordinates back to pixel scale

    float px = x / imgScale;
    float py = y / imgScale;

    px -= 0.5;
    py -= 0.5;

    px = constrain( round( px ), 0, img.width );
    py = constrain( round( py ), 0, img.height );

    float value = brightness( img.get((int)px, (int)py) ); 
    return value / 255.0;
  }

//-----------------------------------------------------------------------------
  void rasterizeImage( PImage img, float imgScale )
  {
    PVector lineVector = new PVector( 3, -1 );
    lineVector.normalize();
    float lineSeparation = 2.0;
    float segmentWidth = 0.3;
    float segmentHeight = 2.0;

    _plotShape = new RShape();

    float imgWidth = img.width * imgScale;
    float imgHeight = img.height * imgScale;

    RShape outlineShape = new RShape();
    outlineShape.addChild( RG.getRect( 0, 0, imgWidth, imgHeight ) );

    RShape rasterLines = getRasterLines( outlineShape, lineVector, lineSeparation );

    RShape rasterLines2 = new RShape();
    for ( int i=0; i<rasterLines.countChildren(); i++ )
    {
      // assume it is a line with two points (not an arc or curve)
      RShape shp = rasterLines.children[i];
      RPoint lineStart = shp.getPoint(0.0);
      RPoint lineEnd = shp.getPoint(1.0);

      PVector thisLineVector = new PVector(  lineEnd.x - lineStart.x, lineEnd.y - lineStart.y );
      float thisLineLength = thisLineVector.mag();
      thisLineVector.normalize();
      PVector thisPerpVector = new PVector( thisLineVector.y, -thisLineVector.x );
      thisPerpVector.mult( -1 );
      thisPerpVector.normalize();

      float segmentCount = thisLineLength / segmentWidth;
      RG.beginShape();
      RG.vertex( lineStart.x, lineStart.y );
      for ( int j=0; j<(int)segmentCount; j++ )
      {
        PVector p0 = PVector.mult( thisLineVector, ((float)j+0.0)*segmentWidth );
        PVector p1 = PVector.mult( thisLineVector, ((float)j+0.25)*segmentWidth );
        PVector p2 = PVector.mult( thisLineVector, ((float)j+0.5)*segmentWidth );
        PVector p3 = PVector.mult( thisLineVector, ((float)j+0.75)*segmentWidth );
        PVector p4 = PVector.mult( thisLineVector, ((float)j+1.0)*segmentWidth );

        p0.add( lineStart.x, lineStart.y, 0 );
        p1.add( lineStart.x, lineStart.y, 0 );
        p2.add( lineStart.x, lineStart.y, 0 );
        p3.add( lineStart.x, lineStart.y, 0 );
        p4.add( lineStart.x, lineStart.y, 0 );

        float darkness = 1.0 - getPixelBrightness( img, imgScale, p2.x, p2.y );

        p1.add( PVector.mult( thisPerpVector, 0.5 * darkness * segmentHeight ) );
        p3.add( PVector.mult( thisPerpVector, -0.5 * darkness * segmentHeight ) );

        //RG.vertex( p0.x, p0.y );
        RG.vertex( p1.x, p1.y );
        //RG.vertex( p2.x, p2.y );
        RG.vertex( p3.x, p3.y );
        RG.vertex( p4.x, p4.y );
      }
      //RG.endShape();

      rasterLines2.addChild( RG.getShape() );
    }

    //_plotShape.addChild( outlineShape );
    //_plotShape.addChild( rasterLines );
    _plotShape.addChild( rasterLines2 );
  }


//-----------------------------------------------------------------------------
  // static
  RShape flattenShape( RShape shape )
  {
    RShape flattenedShape = new RShape();
    _flattenShape( shape, flattenedShape );
    return flattenedShape;
  }

//-----------------------------------------------------------------------------
  // static
  void _flattenShape( RShape sourceShape, RShape targetShape )
  {
    if ( sourceShape == null )
    {
      return;
    }
    if ( sourceShape.countChildren() == 0 )
    { 
      if ( sourceShape.countPaths() > 0 )  // filter out empty layers 
      {
        // single shape with no children
        // put here a filter on shape state
        targetShape.addChild( new RShape( sourceShape ) );
        //println( "shape "+sourceShape.name+" has countPaths="+sourceShape.countPaths() );
        //RPoint center = sourceShape.getCenter();
        //float w = sourceShape.getWidth();
        //float h = sourceShape.getWidth();
        //println( "shape "+sourceShape.name+" at center "+center.x+","+center.y+" size="+w+","+h );
      }
      else
      {
        //println( "shape "+sourceShape.name+" has no paths, ignoring" );
      }
    }
    else
    {
      // shape with children
      // put here a filter on a collection, like layer
      for ( int i=0; i<sourceShape.countChildren(); i++ )
      {
        _flattenShape( sourceShape.children[i], targetShape );
      }
    }
  }

//-----------------------------------------------------------------------------
  // static
  RPoint[][] getPointsInPaths( RShape shape, PVector shapeOrigin )
  {
    if ( true )
    {
      RG.setPolygonizer(RG.ADAPTATIVE);
      //RG.setPolygonizerAngle( radians( 90 ) );
    }
    else
    {
      RG.setPolygonizer(RG.UNIFORMLENGTH);
      RG.setPolygonizerLength(1);
    }

    if ( shape != null )
    {
      RShape translatedShape = new RShape(shape);
      translatedShape.translate( shapeOrigin.x, shapeOrigin.y ); 
      return translatedShape.getPointsInPaths();
    }
    return null;
  }

//-----------------------------------------------------------------------------
  void sortIntersections( RPoint[] intersectPoints, RPoint insertsectLineStart, RPoint insertsectLineEnd )
  {
    if (intersectPoints != null)
    {
      float lineLength = dist( insertsectLineStart.x, insertsectLineStart.y, insertsectLineEnd.x, insertsectLineEnd.y );
      float t[] = new float[ intersectPoints.length ];
      for ( int k=0; k<intersectPoints.length; k++)
      {
        t[k] = dist( intersectPoints[k].x, intersectPoints[k].y, insertsectLineStart.x, insertsectLineStart.y ) / lineLength;
      }

      t = sort( t );

      for ( int k=0; k<intersectPoints.length; k++)
      {
        intersectPoints[k].x = lerp( insertsectLineStart.x, insertsectLineEnd.x, t[k] );
        intersectPoints[k].y = lerp( insertsectLineStart.y, insertsectLineEnd.y, t[k] );
      }
    }
  }

//-----------------------------------------------------------------------------
  RShape getRasterLines( RShape sourceShape, PVector lineVector, float lineSeparation )
  {
    // IMPORTANT! getIntersections() uses segmented paths.
    // Other polygonizer settings cause artifacts,
    // i.e. raster lines won't start/end at shape outline
    RG.setPolygonizer(RG.ADAPTATIVE);

    PVector perpVector = new PVector( lineVector.y, -lineVector.x );
    perpVector.mult( -1 );
    lineVector.normalize();
    perpVector.normalize();

    PVector shapeDiagonal = new PVector( sourceShape.getWidth(), sourceShape.getHeight() );

    PVector topLeft = new PVector( sourceShape.getX(), sourceShape.getY() );
    PVector bottomRight = new PVector( topLeft.x + sourceShape.getWidth(), topLeft.y + sourceShape.getHeight() );

    PVector perpVectorRange = perpVector.get();
    perpVectorRange.mult( shapeDiagonal.dot( perpVector ) );

    float lineCount = perpVectorRange.mag() / lineSeparation;

    RShape targetShape = new RShape();      
    for ( int i=0; i<=lineCount; i++ )
    {
      PVector p = perpVector.get();
      p.mult( (float)i * lineSeparation );

      PVector q1 = lineVector.get();
      q1.mult( -shapeDiagonal.mag() );
      PVector q2 = lineVector.get();
      q2.mult( shapeDiagonal.mag() );

      RPoint insertsectLineStart = new RPoint( sourceShape.getX() + p.x + q1.x, sourceShape.getY() + p.y + q1.y );
      RPoint insertsectLineEnd = new RPoint( sourceShape.getX() + p.x + q2.x, sourceShape.getY() + p.y + q2.y );                                    
      RShape insertsectLine = RG.getLine( insertsectLineStart.x, insertsectLineStart.y, insertsectLineEnd.x, insertsectLineEnd.y );

      //if ( true )
      //{ targetShape.addChild( rasterLine ); continue; }

      RPoint[] ps = sourceShape.getIntersections(insertsectLine);
      if (ps == null)
      {
        continue;
      }

      // this magic solves issue of shapes having multiple paths
      // and the intersections being returned out of order
      // along the insersection line
      sortIntersections( ps, insertsectLineStart, insertsectLineEnd );

      if ( ps.length % 2 == 0 )
      {
        for (int k=0; k+1<ps.length; k+=2)
        {
          if ( ps[k].x == ps[k+1].x && ps[k].y == ps[k+1].y )
          {
            // identical intersects. wierdness.
            // maybe passed through path start and end node?
            k-=1;
            continue;
          }
          // swap direction each time to minimize joins
          RShape newLine;
          if ( i % 2 == 0 )
          {
            newLine = RG.getLine( ps[k].x, ps[k].y, ps[k+1].x, ps[k+1].y );
          }
          else
          {
            newLine = RG.getLine( ps[k+1].x, ps[k+1].y, ps[k].x, ps[k].y );
          }
          targetShape.addChild( newLine );
        }
      }
    }   

    return targetShape;
  }


//-----------------------------------------------------------------------------
  // static
  RShape rasterizeShape( RShape sourceShape )
  {
    // IMPORTANT! getIntersections() uses segmented paths.
    // Other polygonizer settings cause artifacts,
    // i.e. raster lines won't start/end at shape outline
    RG.setPolygonizer(RG.ADAPTATIVE);

    RShape targetShape = new RShape();
    for ( int i=0; i<sourceShape.countChildren(); i++ )
    {
      RShape shp = sourceShape.children[i];

      float w = shp.getWidth();
      float h = shp.getHeight();
      RPoint c = shp.getCenter();

      float x0 = c.x - w;
      float x1 = c.x + w;
      float y0 = c.y - h;
      float y1 = c.y + h;
      int steps = (int)( y1 - y0 );
      steps *= 2;

      int intersectingLineNumber = 0;
      for ( int j=0; j<= steps; j++ )
      {

        int k;
        float y = lerp( y0, y1, (float)j / (float)steps );
        RShape cuttingLine = RG.getLine( x0, y, x1, y );
        float lineLength = dist( x0, y, x1, y );

        RPoint[] ps = shp.getIntersections(cuttingLine);
        if (ps != null)
        {
          /*print( "STEP: "+intersectingLineNumber++ +"  ");
           print("points=");
           for (k=0; k<ps.length; k++)
           {
           print(" "+ps[k].x+","+ps[k].y);
           }
           println();*/

          float intersects[] = new float[ ps.length ];
          for ( k=0; k<ps.length; k++)
          {
            intersects[k] = dist( ps[k].x, ps[k].y, x0, y ) / lineLength;
          }

          intersects = sort( intersects );

          for ( k=0; k<ps.length; k++)
          {
            ps[k].x = lerp( x0, x1, intersects[k] );
            ps[k].y = lerp( y, y, intersects[k] );
          }

          if ( ps.length % 2 == 0 )
          {
            for (k=0; k+1<ps.length; k+=2)
            {
              if ( ps[k].x == ps[k+1].x && ps[k].y == ps[k+1].y )
              {
                // identical intersects. wierdness.
                // maybe passed through path start and end node?
                k-=1;
                continue;
              }
              // swap direction each time to minimize joins
              RShape newLine;
              if ( j % 2 == 0 )
              {
                newLine = RG.getLine( ps[k].x, ps[k].y, ps[k+1].x, ps[k+1].y );
              }
              else
              {
                newLine = RG.getLine( ps[k+1].x, ps[k+1].y, ps[k].x, ps[k].y );
              }
              targetShape.addChild( newLine );
            }
          }
        }
      }
    }
    return targetShape;
  }
}

