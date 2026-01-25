
  public static class Paths {
    public PathChain Path1;
public PathChain Path2;
public PathChain Path3;
public PathChain Path4;
public PathChain Path5;
public PathChain Path6;
public PathChain Path7;
public PathChain Path8;
public PathChain Path9;
public PathChain Path10;
    
    public Paths(Follower follower) {
      Path1 = follower.pathBuilder().addPath(
          new BezierCurve(
            new Pose(88.000, 8.000),
            new Pose(93.743, 37.701),
            new Pose(103.000, 40.000)
          )
        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
        
        .build();

Path2 = follower.pathBuilder().addPath(
          new BezierLine(
            new Pose(103.000, 40.000),
            
            new Pose(125.000, 40.000)
          )
        ).setTangentHeadingInterpolation()
        .setReversed(true)
        .build();

Path3 = follower.pathBuilder().addPath(
          new BezierCurve(
            new Pose(125.000, 40.000),
            new Pose(97.075, 42.664),
            new Pose(80.500, 15.000)
          )
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
        
        .build();

Path4 = follower.pathBuilder().addPath(
          new BezierCurve(
            new Pose(80.500, 15.000),
            new Pose(91.890, 60.047),
            new Pose(103.000, 64.000)
          )
        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
        
        .build();

Path5 = follower.pathBuilder().addPath(
          new BezierLine(
            new Pose(103.000, 64.000),
            
            new Pose(128.000, 64.000)
          )
        ).setTangentHeadingInterpolation()
        .setReversed(true)
        .build();

Path6 = follower.pathBuilder().addPath(
          new BezierCurve(
            new Pose(128.000, 64.000),
            new Pose(96.708, 62.930),
            new Pose(80.500, 15.000)
          )
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
        
        .build();

Path7 = follower.pathBuilder().addPath(
          new BezierCurve(
            new Pose(80.500, 15.000),
            new Pose(87.507, 74.430),
            new Pose(103.000, 87.000)
          )
        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
        
        .build();

Path8 = follower.pathBuilder().addPath(
          new BezierLine(
            new Pose(103.000, 87.000),
            
            new Pose(125.766, 87.000)
          )
        ).setTangentHeadingInterpolation()
        .setReversed(true)
        .build();

Path9 = follower.pathBuilder().addPath(
          new BezierCurve(
            new Pose(125.766, 87.000),
            new Pose(92.797, 87.234),
            new Pose(80.500, 15.000)
          )
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
        
        .build();

Path10 = follower.pathBuilder().addPath(
          new BezierLine(
            new Pose(80.500, 15.000),
            
            new Pose(80.500, 42.000)
          )
        ).setTangentHeadingInterpolation()
        
        .build();
    }
  }