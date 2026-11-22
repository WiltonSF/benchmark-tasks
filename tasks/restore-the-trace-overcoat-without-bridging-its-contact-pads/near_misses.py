"""Pose-only shortcuts cannot produce material or a known release history."""
import task
CASES=[('no-op',{},0.,False),('brush over target but dry and no history',{'mg859_brush':task.BRUSH.state(pos=(.30,.04,task.TOP+.06))},0.,False),
 ('board moved without coating',{'crickit_controller':task.BOARD.state(pos=(.33,.02,task.TOP+.033))},0.,False),
 ('upside down controller',{'crickit_controller':task.BOARD.state(quat_xyzw=(1.,0.,0.,0.))},0.,False),
 ('released tool alone',{'mg859_brush':task.BRUSH.state()},0.,False)]
