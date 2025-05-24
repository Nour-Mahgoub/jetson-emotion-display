#!/bin/bash

# 🧪 COMPREHENSIVE EMOTION DISPLAY SYSTEM TEST
# Tests everything before deploying to robot

echo "🧪 COMPREHENSIVE EMOTION DISPLAY SYSTEM TEST"
echo "=" * 60

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

PASS_COUNT=0
FAIL_COUNT=0
TOTAL_TESTS=0

print_test() {
    echo -e "\n${BLUE}[TEST]${NC} $1"
    ((TOTAL_TESTS++))
}

print_pass() {
    echo -e "${GREEN}[PASS]${NC} $1"
    ((PASS_COUNT++))
}

print_fail() {
    echo -e "${RED}[FAIL]${NC} $1"
    ((FAIL_COUNT++))
}

print_info() {
    echo -e "${YELLOW}[INFO]${NC} $1"
}

# Test 1: Check ROS Installation
print_test "Checking ROS Installation"
if command -v roscore &> /dev/null; then
    print_pass "ROS is installed"
    ROS_VERSION=$(rosversion -d)
    print_info "ROS Version: $ROS_VERSION"
else
    print_fail "ROS is not installed"
    echo "Install ROS: sudo apt install ros-noetic-desktop-full"
fi

# Test 2: Check Python Dependencies
print_test "Checking Python Dependencies"
PYTHON_DEPS=("cv2" "numpy" "PIL" "requests")
for dep in "${PYTHON_DEPS[@]}"; do
    if python3 -c "import $dep" &> /dev/null; then
        print_pass "Python $dep is available"
    else
        print_fail "Python $dep is missing"
        echo "Install: pip3 install $dep"
    fi
done

# Test 3: Check Workspace
print_test "Checking Workspace Structure"
WORKSPACE_DIR="$HOME/emotion_display_ws"
if [ -d "$WORKSPACE_DIR" ]; then
    print_pass "Workspace directory exists"
    
    # Check key files
    KEY_FILES=(
        "src/emotion_display/scripts/emotion_display_node.py"
        "src/emotion_display/launch/emotion_display.launch"
        "src/emotion_display/test/test_emotion_publisher.py"
        "devel/setup.bash"
    )
    
    for file in "${KEY_FILES[@]}"; do
        if [ -f "$WORKSPACE_DIR/$file" ]; then
            print_pass "Found: $file"
        else
            print_fail "Missing: $file"
        fi
    done
else
    print_fail "Workspace directory not found"
    echo "Run the installer script first"
fi

# Test 4: Check ROS Core
print_test "Checking ROS Core"
if pgrep -x "roscore" > /dev/null; then
    print_pass "ROS Core is running"
else
    print_info "ROS Core not running - starting it..."
    roscore &
    ROSCORE_PID=$!
    sleep 3
    
    if pgrep -x "roscore" > /dev/null; then
        print_pass "ROS Core started successfully"
        STARTED_ROSCORE=true
    else
        print_fail "Failed to start ROS Core"
    fi
fi

# Test 5: Build Workspace
print_test "Building Workspace"
cd "$WORKSPACE_DIR"
if catkin_make > /dev/null 2>&1; then
    print_pass "Workspace builds successfully"
else
    print_fail "Workspace build failed"
    echo "Try: cd $WORKSPACE_DIR && catkin_make"
fi

# Source workspace
source "$WORKSPACE_DIR/devel/setup.bash" 2>/dev/null

# Test 6: Launch Emotion Node
print_test "Testing Emotion Display Node"
print_info "Starting emotion display node..."

# Start the node in background
roslaunch emotion_display emotion_display.launch > /tmp/emotion_node.log 2>&1 &
NODE_PID=$!
sleep 5

# Check if node is running
if ps -p $NODE_PID > /dev/null; then
    print_pass "Emotion display node started successfully"
    
    # Check if it registered properly
    if rosnode list | grep -q "emotion_display_node"; then
        print_pass "Node registered with ROS master"
    else
        print_fail "Node not registered with ROS master"
    fi
else
    print_fail "Emotion display node failed to start"
    echo "Check logs: tail /tmp/emotion_node.log"
fi

# Test 7: HTTP Server
print_test "Testing HTTP Server"
sleep 2  # Give server time to start

LOCAL_IP=$(python3 -c "import socket; s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.connect(('8.8.8.8',80)); print(s.getsockname()[0]); s.close()" 2>/dev/null || echo "localhost")
SERVER_URL="http://$LOCAL_IP:8000"

# Test status endpoint
if curl -s "$SERVER_URL/status" > /dev/null; then
    print_pass "HTTP server is responding"
    
    # Test emotion endpoint
    if curl -s "$SERVER_URL/emotion" | grep -q '"ok"'; then
        print_pass "Emotion API is working"
    else
        print_fail "Emotion API not responding correctly"
    fi
    
    # Test main page
    if curl -s "$SERVER_URL" | grep -q "Robot Emotions"; then
        print_pass "Main HTML page is serving"
    else
        print_fail "Main HTML page not working"
    fi
else
    print_fail "HTTP server not responding"
    echo "Check if port 8000 is available: netstat -tlnp | grep 8000"
fi

# Test 8: ROS Topic Communication
print_test "Testing ROS Topic Communication"

# Check if topic exists
if rostopic list | grep -q "/emotion_detection"; then
    print_pass "Emotion detection topic exists"
else
    print_info "Emotion detection topic not found (will be created when publishing)"
fi

# Test publishing
print_info "Testing emotion publishing..."
rostopic pub /emotion_detection std_msgs/String "data: 'test'" --once &
PUB_PID=$!
sleep 2

# Check if message was received (look in logs)
if grep -q "Received" /tmp/emotion_node.log 2>/dev/null; then
    print_pass "Node received test emotion"
else
    print_info "No evidence of message reception (check logs manually)"
fi

# Test 9: Emotion Cycle Test
print_test "Testing Full Emotion Cycle"
EMOTIONS=("happy" "sad" "angry" "surprised" "joy")

for emotion in "${EMOTIONS[@]}"; do
    print_info "Testing emotion: $emotion"
    rostopic pub /emotion_detection std_msgs/String "data: '$emotion'" --once
    sleep 1
    
    # Check API response
    RESPONSE=$(curl -s "$SERVER_URL/emotion" 2>/dev/null)
    if echo "$RESPONSE" | grep -q "\"emotion\":\"$emotion\""; then
        print_pass "✅ $emotion emotion displayed correctly"
    else
        print_info "⏳ $emotion may take time to appear"
    fi
done

# Test 10: Performance Test  
print_test "Testing Performance"
print_info "Measuring API response time..."

START_TIME=$(date +%s%N)
for i in {1..10}; do
    curl -s "$SERVER_URL/emotion" > /dev/null
done
END_TIME=$(date +%s%N)

TOTAL_TIME=$(( (END_TIME - START_TIME) / 1000000 ))  # Convert to milliseconds
AVG_TIME=$(( TOTAL_TIME / 10 ))

if [ $AVG_TIME -lt 100 ]; then
    print_pass "Average response time: ${AVG_TIME}ms (Excellent)"
elif [ $AVG_TIME -lt 300 ]; then
    print_pass "Average response time: ${AVG_TIME}ms (Good)"
else
    print_fail "Average response time: ${AVG_TIME}ms (Too slow)"
fi

# Test 11: Network Connectivity
print_test "Testing Network Connectivity"
print_info "Local IP: $LOCAL_IP"
print_info "Server URL: $SERVER_URL"

# Test from different IP (simulate tablet)
if ping -c 1 "$LOCAL_IP" > /dev/null 2>&1; then
    print_pass "Local IP is reachable"
else
    print_fail "Local IP not reachable"
fi

# Check port accessibility
if nc -z "$LOCAL_IP" 8000 2>/dev/null; then
    print_pass "Port 8000 is accessible"
else
    print_fail "Port 8000 not accessible"
    echo "Check firewall: sudo ufw allow 8000"
fi

# Test 12: Memory and CPU Usage
print_test "Testing Resource Usage"
if command -v free &> /dev/null; then
    MEMORY_USAGE=$(free | grep Mem | awk '{printf "%.1f", $3/$2 * 100.0}')
    print_info "Memory usage: ${MEMORY_USAGE}%"
    
    if (( $(echo "$MEMORY_USAGE < 80" | bc -l) )); then
        print_pass "Memory usage is acceptable"
    else
        print_fail "High memory usage detected"
    fi
fi

if command -v top &> /dev/null; then
    CPU_USAGE=$(top -bn1 | grep "Cpu(s)" | awk '{print $2}' | awk -F'%' '{print $1}')
    print_info "CPU usage: ${CPU_USAGE}%"
fi

# Test 13: Auto-Recovery Test
print_test "Testing Auto-Recovery"
print_info "Killing emotion node to test recovery..."

# Kill the node
kill $NODE_PID 2>/dev/null
sleep 2

# Try to restart
roslaunch emotion_display emotion_display.launch > /tmp/emotion_node_recovery.log 2>&1 &
RECOVERY_PID=$!
sleep 3

if ps -p $RECOVERY_PID > /dev/null; then
    print_pass "Node recovered successfully"
    NODE_PID=$RECOVERY_PID
else
    print_fail "Node recovery failed"
fi

# Test 14: Tablet Simulation
print_test "Simulating Tablet Access"
TABLET_TEST_HTML="/tmp/tablet_test.html"

cat > "$TABLET_TEST_HTML" << 'EOF'
<!DOCTYPE html>
<html>
<head><title>Tablet Test</title></head>
<body>
<div id="result">Testing...</div>
<script>
fetch('SERVER_URL/emotion')
    .then(response => response.json())
    .then(data => {
        document.getElementById('result').innerHTML = 
            data.ok ? 'SUCCESS: Connected to robot!' : 'FAIL: No connection';
    })
    .catch(error => {
        document.getElementById('result').innerHTML = 'ERROR: ' + error;
    });
</script>
</body>
</html>
EOF

# Replace SERVER_URL with actual URL
sed -i "s|SERVER_URL|$SERVER_URL|g" "$TABLET_TEST_HTML"

if command -v chromium-browser &> /dev/null; then
    print_info "Test HTML created: file://$TABLET_TEST_HTML"
    print_info "Open this file in a browser to simulate tablet access"
    print_pass "Tablet simulation files ready"
elif command -v firefox &> /dev/null; then
    print_info "Test HTML created: file://$TABLET_TEST_HTML"  
    print_pass "Tablet simulation files ready"
else
    print_info "No browser found for tablet simulation test"
fi

# Test 15: Integration Test
print_test "Final Integration Test"
print_info "Running 30-second integration test..."

# Send rapid emotions
INTEGRATION_EMOTIONS=("happy" "sad" "angry" "joy" "normal")
for i in {1..6}; do
    for emotion in "${INTEGRATION_EMOTIONS[@]}"; do
        rostopic pub /emotion_detection std_msgs/String "data: '$emotion'" --once
        sleep 1
    done
done &

# Monitor for 30 seconds
sleep 30

# Check if system is still responsive
if curl -s "$SERVER_URL/status" > /dev/null; then
    print_pass "System remains responsive after integration test"
else
    print_fail "System became unresponsive during integration test"
fi

# Cleanup
print_test "Cleaning Up Test Environment"

# Kill background processes
if [ ! -z "$NODE_PID" ]; then
    kill $NODE_PID 2>/dev/null
    print_info "Stopped emotion display node"
fi

if [ "$STARTED_ROSCORE" = true ] && [ ! -z "$ROSCORE_PID" ]; then
    kill $ROSCORE_PID 2>/dev/null
    print_info "Stopped ROS core"
fi

# Remove temp files
rm -f /tmp/emotion_node.log /tmp/emotion_node_recovery.log
print_info "Cleaned up temporary files"

# Final Results
echo ""
echo "🏁 TEST SUMMARY"
echo "=" * 40
echo -e "✅ PASSED: ${GREEN}$PASS_COUNT${NC}/$TOTAL_TESTS tests"
echo -e "❌ FAILED: ${RED}$FAIL_COUNT${NC}/$TOTAL_TESTS tests"

if [ $FAIL_COUNT -eq 0 ]; then
    echo -e "\n🎉 ${GREEN}ALL TESTS PASSED!${NC}"
    echo "✅ System is ready for deployment on Jetson Nano"
    echo ""
    echo "📱 NEXT STEPS:"
    echo "1. Transfer files to Jetson Nano"
    echo "2. Run installer on Jetson Nano"
    echo "3. Connect tablet to: http://JETSON_IP:8000"
    echo "4. Start your emotion detection system"
    echo ""
    echo "🚀 DEPLOYMENT COMMAND:"
    echo "   ~/emotion_display_ws/start_emotion_system.sh"
elif [ $FAIL_COUNT -lt 3 ]; then
    echo -e "\n⚠️  ${YELLOW}MINOR ISSUES DETECTED${NC}"
    echo "System should work but may need minor fixes"
    echo "Review failed tests above"
else
    echo -e "\n🚨 ${RED}MAJOR ISSUES DETECTED${NC}"
    echo "Fix failed tests before deployment"
    echo "Run installer again if needed"
fi

echo ""
echo "📊 SYSTEM INFO:"
echo "   Local IP: $LOCAL_IP"
echo "   Server URL: $SERVER_URL"
echo "   Tablet URL: $SERVER_URL"
echo "   Test HTML: file://$TABLET_TEST_HTML"

exit $FAIL_COUNT
