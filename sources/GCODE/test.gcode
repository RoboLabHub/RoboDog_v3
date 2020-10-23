$PARAM_RESET
#W 10000

# S 200
# $POSE 0

# S 100
# $POSE 3
# W 5000

# SAY en I am RoboDog! Nice to meet you!
# PLAY_SOUND ~/barking.wav

S 200
$POSE 5
#W 500

# $WALK_2 3 1
# W 500

# $WALK_2 3 0
# W 500

# S 50
# BASE_ROT 0 0 30
# $POSE 5
# W 1000

# BASE_ROT 0 0 -30
# $POSE 5
# W 1000

# S 100
# BASE_ROT 0 0 0
# $POSE 3
# W 2000

# S 250
# $ROTATE 3 0
# W 1000

# S 250
# $ROTATE 6 1
# W 1000

# S 250
# $ROTATE 3 0
# W 1000

S 200

M+ -10 -80 0
W 500
$WALK_2 20 1

# M+ -10 -80 0
# W 500
# $WALK_2 20 0

W 500
$POSE 0
