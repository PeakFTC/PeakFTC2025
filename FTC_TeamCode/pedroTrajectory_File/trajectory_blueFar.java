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
            new Pose(58.000, 9.000),
            new Pose(68.860, 22.654),
            new Pose(44.860, 39.477)
          )
        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
        
        .build();

Path2 = follower.pathBuilder().addPath(
          new BezierLine(
            new Pose(44.860, 39.477),
            
            new Pose(19.738, 40.150)
          )
        ).setTangentHeadingInterpolation()
        .setReversed(true)
        .build();

Path3 = follower.pathBuilder().addPath(
          new BezierCurve(
            new Pose(19.738, 40.150),
            new Pose(67.514, 34.318),
            new Pose(58.000, 15.000)
          )
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
        
        .build();

Path4 = follower.pathBuilder().addPath(
          new BezierCurve(
            new Pose(58.000, 15.000),
            new Pose(58.318, 56.075),
            new Pose(43.963, 63.477)
          )
        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
        
        .build();

Path5 = follower.pathBuilder().addPath(
          new BezierLine(
            new Pose(43.963, 63.477),
            
            new Pose(14.080, 63.070)
          )
        ).setTangentHeadingInterpolation()
        .setReversed(true)
        .build();

Path6 = follower.pathBuilder().addPath(
          new BezierCurve(
            new Pose(14.080, 63.070),
            new Pose(60.336, 62.579),
            new Pose(57.869, 15.028)
          )
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
        
        .build();

Path7 = follower.pathBuilder().addPath(
          new BezierCurve(
            new Pose(57.869, 15.028),
            new Pose(66.168, 68.636),
            new Pose(43.065, 87.028)
          )
        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
        
        .build();

Path8 = follower.pathBuilder().addPath(
          new BezierLine(
            new Pose(43.065, 87.028),
            
            new Pose(16.598, 87.925)
          )
        ).setTangentHeadingInterpolation()
        .setReversed(true)
        .build();

Path9 = follower.pathBuilder().addPath(
          new BezierCurve(
            new Pose(16.598, 87.925),
            new Pose(64.823, 52.262),
            new Pose(57.645, 15.028)
          )
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
        .setReversed(true)
        .build();

Path10 = follower.pathBuilder().addPath(
          new BezierLine(
            new Pose(57.645, 15.028),
            
            new Pose(57.383, 32.439)
          )
        ).setTangentHeadingInterpolation()
        
        .build();
    }
  }
  