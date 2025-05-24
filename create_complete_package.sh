#!/bin/bash

# 📦 CREATE COMPLETE JETSON NANO EMOTION DISPLAY PACKAGE
# This script creates everything you need in one downloadable package

echo "📦 CREATING COMPLETE JETSON NANO EMOTION DISPLAY PACKAGE"
echo "This will create everything you need for your robot!"
echo "=" * 60

# Package configuration
PACKAGE_NAME="jetson_emotion_display_complete"
PACKAGE_DIR="/tmp/$PACKAGE_NAME" 
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
ARCHIVE_NAME="${PACKAGE_NAME}_${TIMESTAMP}.tar.gz"

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m'

print_status() { echo -e "${BLUE}[CREATE]${NC} $1"; }
print_success() { echo -e "${GREEN}[DONE]${NC} $1"; }
print_info() { echo -e "${YELLOW}[INFO]${NC} $1"; }

# Clean and create package directory
rm -rf "$PACKAGE_DIR"
mkdir -p "$PACKAGE_DIR"

print_status "Creating package structure..."

# Create all directories
mkdir -p "$PACKAGE_DIR/emotion_display_ws/src/emotion_display/scripts"
mkdir -p "$PACKAGE_DIR/emotion_display_ws/src/emotion_display/launch" 
mkdir -p "$PACKAGE_DIR/emotion_display_ws/src/emotion_display/test"
mkdir -p "$PACKAGE_DIR/emotion_display_ws/src/emotion_display/examples"
mkdir -p "$PACKAGE_DIR/docs"
mkdir -p "$PACKAGE_DIR/tools"

print_success "Package structure created"

# 1. MAIN INSTALLER SCRIPT
print_status "Creating main installer..."
cat > "$PACKAGE_DIR/INSTALL_ON_JETSON.sh" << 'INSTALLER_EOF'
#!/bin/bash

# 🚀 JETSON NANO EMOTION DISPLAY - ONE-CLICK INSTALLER
echo "🤖 JETSON NANO EMOTION DISPLAY SYSTEM"
echo "One-click installer for robot emotion display on tablets"
echo "=" * 50

# Colors
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; BLUE='\033[0;34m'; NC='\033[0m'
print_status() { echo -e "${BLUE}[INFO]${NC} $1"; }
print_success() { echo -e "${GREEN}[OK]${NC} $1"; }
print_error() { echo -e "${RED}[ERROR]${NC} $1"; }

# Check system
if [[ $EUID -eq 0 ]]; then print_error "Don't run as root"; exit 1; fi

print_status "Installing on Jetson Nano..."
WORKSPACE_DIR="$HOME/emotion_display_ws"

# Update system
print_status "Updating system packages..."
sudo apt-get update -qq

# Install dependencies  
print_status "Installing dependencies..."
sudo apt-get install -y python3-pip python3-opencv python3-numpy python3-pil \
    ros-noetic-cv-bridge ros-noetic-std-msgs ros-noetic-sensor-msgs \
    ros-noetic-rospy curl wget netcat bc

pip3 install --user pillow requests opencv-python

# Create workspace
print_status "Setting up workspace..."
if [ -d "$WORKSPACE_DIR" ]; then
    mv "$WORKSPACE_DIR" "$WORKSPACE_DIR.backup.$(date +%Y%m%d_%H%M%S)"
fi

# Copy files
cp -r emotion_display_ws "$HOME/"
chmod +x "$WORKSPACE_DIR/src/emotion_display/scripts/"*.py
chmod +x "$WORKSPACE_DIR/src/emotion_display/test/"*.py

# Build workspace
print_status "Building ROS workspace..."
cd "$WORKSPACE_DIR"
source /opt/ros/noetic/setup.bash
catkin_make

# Create startup script
cat > "$WORKSPACE_DIR/start_system.sh" << 'START_EOF'
#!/bin/bash
echo "🤖 Starting Jetson Nano Emotion Display..."
source /opt/ros/noetic/setup.bash
source ~/emotion_display_ws/devel/setup.bash

LOCAL_IP=$(python3 -c "import socket; s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.connect(('8.8.8.8',80)); print(s.getsockname()[0]); s.close()")
echo "🌐 Jetson IP: $LOCAL_IP"
echo "📱 Tablet URL: http://$LOCAL_IP:8000"
echo ""

if ! pgrep -x "roscore" > /dev/null; then
    echo "🚀 Starting ROS Core..."
    roscore &
    sleep 3
fi

echo "🎭 Starting Emotion Display..."
echo "✅ Ready! Connect tablet to: http://$LOCAL_IP:8000"
roslaunch emotion_display emotion_display.launch
START_EOF

chmod +x "$WORKSPACE_DIR/start_system.sh"

# Add to bashrc
if ! grep -q "emotion_display_ws" ~/.bashrc; then
    echo "source $WORKSPACE_DIR/devel/setup.bash" >> ~/.bashrc
fi

# Get IP for final message
LOCAL_IP=$(python3 -c "import socket; s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.connect(('8.8.8.8',80)); print(s.getsockname()[0]); s.close()" 2>/dev/null || echo "unknown")

echo ""
print_success "INSTALLATION COMPLETE!"
echo "=" * 40
echo "🚀 START: $WORKSPACE_DIR/start_system.sh"
echo "📱 TABLET: http://$LOCAL_IP:8000"
echo "🧪 TEST: rostopic pub /emotion_detection std_msgs/String \"data: 'happy'\""
echo "=" * 40
INSTALLER_EOF

chmod +x "$PACKAGE_DIR/INSTALL_ON_JETSON.sh"
print_success "Main installer created"

# 2. EMOTION DISPLAY NODE
print_status "Creating emotion display node..."
cat > "$PACKAGE_DIR/emotion_display_ws/src/emotion_display/scripts/emotion_display_node.py" << 'NODE_EOF'
#!/usr/bin/env python3
"""🤖 JETSON NANO EMOTION DISPLAY NODE - PRODUCTION VERSION"""

import rospy, json, time, threading, base64, io, socket
from std_msgs.msg import String
from http.server import HTTPServer, BaseHTTPRequestHandler
from PIL import Image, ImageDraw, ImageFont

class JetsonEmotionNode:
    def __init__(self):
        rospy.init_node('emotion_display_node', anonymous=True)
        
        # State management
        self.current_emotion = "normal"
        self.emotion_start_time = 0
        self.emotion_duration = 60  # 1 minute
        self.emotion_count = 0
        self.latest_image = None
        self.lock = threading.Lock()
        
        # Configuration
        self.emotion_topic = rospy.get_param('~emotion_topic', '/emotion_detection')
        self.http_port = rospy.get_param('~http_port', 8000)
        self.local_ip = self.get_local_ip()
        
        print(f"🤖 Jetson Nano Emotion Display Starting...")
        print(f"📡 Topic: {self.emotion_topic}")
        print(f"🌐 Server: http://{self.local_ip}:{self.http_port}")
        
        # Subscribe to emotions
        self.emotion_sub = rospy.Subscriber(self.emotion_topic, String, self.emotion_callback, queue_size=1)
        
        # Create initial image
        self.update_emotion_image()
        
        # Start HTTP server
        self.start_http_server()
        print("✅ Jetson Emotion Node Ready!")
        
    def get_local_ip(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except: return "localhost"
    
    def emotion_callback(self, msg):
        emotion = msg.data.lower().strip()
        current_time = time.time()
        
        with self.lock:
            if emotion != self.current_emotion:
                self.current_emotion = emotion
                self.emotion_start_time = current_time
                self.emotion_count += 1
                print(f"🎭 New emotion: {emotion} (#{self.emotion_count})")
                self.update_emotion_image()
                
    def update_emotion_image(self):
        current_time = time.time()
        
        # Check expiration
        if (self.current_emotion != "normal" and 
            current_time - self.emotion_start_time > self.emotion_duration):
            self.current_emotion = "normal"
            print("⏰ Emotion expired - returning to normal")
        
        # Create image
        image = self.create_emotion_image(self.current_emotion)
        with self.lock:
            self.latest_image = self.image_to_base64(image)
    
    def create_emotion_image(self, emotion):
        width, height = 800, 600
        
        # Emotion configs
        configs = {
            "happy": {"color": (50, 205, 50), "emoji": "😊", "text": "HAPPY"},
            "sad": {"color": (30, 144, 255), "emoji": "😢", "text": "SAD"},
            "angry": {"color": (220, 20, 60), "emoji": "😠", "text": "ANGRY"},
            "surprised": {"color": (255, 215, 0), "emoji": "😲", "text": "SURPRISED"},
            "joy": {"color": (255, 69, 0), "emoji": "😄", "text": "JOY"},
            "fear": {"color": (138, 43, 226), "emoji": "😨", "text": "FEAR"},
            "normal": {"color": (70, 70, 70), "emoji": "🤖", "text": "NORMAL"}
        }
        
        config = configs.get(emotion, configs["normal"])
        
        # Create image
        img = Image.new('RGB', (width, height), color=config["color"])
        draw = ImageDraw.Draw(img)
        
        # Use default fonts
        try:
            font_big = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 80)
            font_emoji = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 180)
            font_small = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 24)
        except:
            font_big = font_emoji = font_small = ImageFont.load_default()
        
        # Draw emoji
        emoji_text = config["emoji"]
        try:
            bbox = draw.textbbox((0, 0), emoji_text, font=font_emoji)
            emoji_x = (width - (bbox[2] - bbox[0])) // 2
            draw.text((emoji_x, 80), emoji_text, fill="white", font=font_emoji)
        except:
            draw.text((width//2 - 50, 80), emoji_text, fill="white", font=font_emoji)
        
        # Draw text
        text = config["text"]
        try:
            bbox = draw.textbbox((0, 0), text, font=font_big)
            text_x = (width - (bbox[2] - bbox[0])) // 2
            draw.text((text_x, 380), text, fill="white", font=font_big)
        except:
            draw.text((width//2 - 100, 380), text, fill="white", font=font_big)
        
        # Draw info
        timestamp = time.strftime("%H:%M:%S")
        time_remaining = max(0, self.emotion_duration - (time.time() - self.emotion_start_time))
        
        if self.current_emotion != "normal" and time_remaining > 0:
            info = f"Time: {timestamp} | Remaining: {int(time_remaining)}s | Count: {self.emotion_count}"
        else:
            info = f"Time: {timestamp} | Count: {self.emotion_count}"
            
        draw.text((20, height - 40), info, fill="white", font=font_small)
        draw.rectangle([(0, 0), (width-1, height-1)], outline="white", width=3)
        
        return img
    
    def image_to_base64(self, pil_image):
        buffer = io.BytesIO()
        pil_image.save(buffer, format='JPEG', quality=85)
        return f"data:image/jpeg;base64,{base64.b64encode(buffer.getvalue()).decode()}"
    
    def start_http_server(self):
        handler = type('Handler', (BaseHTTPRequestHandler,), {
            'node': self, 'do_GET': self.handle_request, 'do_OPTIONS': self.handle_options,
            'log_message': lambda self, *args: None
        })
        
        try:
            server = HTTPServer(('0.0.0.0', self.http_port), handler)
            threading.Thread(target=server.serve_forever, daemon=True).start()
            print(f"🌐 HTTP Server started on port {self.http_port}")
        except Exception as e:
            print(f"❌ HTTP Server error: {e}")
    
    def handle_options(self, request):
        request.send_response(200)
        request.send_header('Access-Control-Allow-Origin', '*')
        request.send_header('Access-Control-Allow-Methods', 'GET, OPTIONS')
        request.end_headers()
    
    def handle_request(self, request):
        try:
            request.send_response(200)
            request.send_header('Access-Control-Allow-Origin', '*')
            request.send_header('Cache-Control', 'no-cache')
            
            if request.path.startswith('/emotion'):
                request.send_header('Content-Type', 'application/json')
                request.end_headers()
                
                time_remaining = max(0, self.emotion_duration - (time.time() - self.emotion_start_time))
                
                with self.lock:
                    response = {
                        'ok': True, 'image': self.latest_image, 'emotion': self.current_emotion,
                        'count': self.emotion_count, 'time_remaining': time_remaining
                    }
                
                request.wfile.write(json.dumps(response).encode())
                
            elif request.path.startswith('/status'):
                request.send_header('Content-Type', 'application/json')
                request.end_headers()
                
                status = {
                    'jetson_ip': self.local_ip, 'active': True, 'emotion': self.current_emotion,
                    'count': self.emotion_count, 'topic': self.emotion_topic
                }
                
                request.wfile.write(json.dumps(status, indent=2).encode())
                
            else:
                request.send_header('Content-Type', 'text/html')
                request.end_headers()
                request.wfile.write(self.get_html().encode())
                
        except Exception as e:
            print(f"HTTP Error: {e}")
            request.send_error(500)
    
    def get_html(self):
        return f'''<!DOCTYPE html>
<html><head><title>🤖 Jetson Nano Emotions</title>
<meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
<style>
    body {{margin:0; background:#000; color:#fff; font-family:Arial; overflow:hidden; height:100vh}}
    #img {{width:100vw; height:100vh; object-fit:contain; display:block}}
    #status {{position:fixed; top:10px; left:10px; background:rgba(0,0,0,0.8); padding:8px 15px; 
             border-radius:15px; font-size:14px; z-index:1000; border:1px solid rgba(255,255,255,0.3)}}
    #timer {{position:fixed; top:10px; right:10px; background:rgba(255,69,0,0.9); padding:8px 15px;
            border-radius:15px; font-size:14px; font-weight:bold; z-index:1000; display:none}}
    .pulse {{animation: pulse 1s infinite}} @keyframes pulse {{0%{{opacity:0.8}} 50%{{opacity:1}} 100%{{opacity:0.8}}}}
</style></head>
<body>
<div id="status">🤖 Jetson Ready</div><div id="timer"></div><img id="img">
<script>
const img=document.getElementById('img'), status=document.getElementById('status'), timer=document.getElementById('timer');
let lastCount=0;
function update(){{
    fetch('/emotion?t='+Date.now()).then(r=>r.json()).then(d=>{{
        if(d.ok && d.image){{
            img.src=d.image; status.textContent=`🎭 ${{d.emotion.toUpperCase()}} (#${{d.count}})`;
            if(d.emotion!=='normal' && d.time_remaining>0){{
                timer.style.display='block'; timer.textContent=`⏰ ${{Math.ceil(d.time_remaining)}}s`;
                timer.className=d.time_remaining<10?'pulse':'';
            }}else timer.style.display='none';
            if(d.count>lastCount){{document.body.style.filter='brightness(1.5)';
                setTimeout(()=>document.body.style.filter='brightness(1)',200); lastCount=d.count;}}
        }}
    }}).catch(()=>status.textContent='❌ Error');
}}
setInterval(update,100); update();
if('wakeLock' in navigator) navigator.wakeLock.request('screen').catch(()=>{{}});
</script></body></html>'''
    
    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.update_emotion_image()
            rate.sleep()

if __name__ == '__main__':
    try:
        node = JetsonEmotionNode()
        node.run()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        print("🛑 Jetson Emotion Node stopped")
NODE_EOF

chmod +x "$PACKAGE_DIR/emotion_display_ws/src/emotion_display/scripts/emotion_display_node.py"
print_success "Emotion display node created"

# 3. ROS PACKAGE FILES
print_status "Creating ROS package files..."

# Package.xml
cat > "$PACKAGE_DIR/emotion_display_ws/src/emotion_display/package.xml" << 'EOF'
<?xml version="1.0"?>
<package format="2">
  <n>emotion_display</n>
  <version>1.0.0</version>
  <description>Jetson Nano Emotion Display for Android Tablets</description>
  <maintainer email="robot@jetson.local">Jetson Team</maintainer>
  <license>MIT</license>
  <buildtool_depend>catkin</buildtool_depend>
  <depend>rospy</depend><depend>std_msgs</depend><depend>sensor_msgs</depend>
</package>
EOF

# CMakeLists.txt
cat > "$PACKAGE_DIR/emotion_display_ws/src/emotion_display/CMakeLists.txt" << 'EOF'
cmake_minimum_required(VERSION 3.0.2)
project(emotion_display)
find_package(catkin REQUIRED COMPONENTS rospy std_msgs sensor_msgs)
catkin_package()
EOF

# Launch file
cat > "$PACKAGE_DIR/emotion_display_ws/src/emotion_display/launch/emotion_display.launch" << 'EOF'
<launch>
    <arg name="emotion_topic" default="/emotion_detection" />
    <arg name="http_port" default="8000" />
    <node pkg="emotion_display" type="emotion_display_node.py" name="emotion_display_node" output="screen">
        <param name="emotion_topic" value="$(arg emotion_topic)" />
        <param name="http_port" value="$(arg http_port)" />
    </node>
</launch>
EOF

print_success "ROS package files created"

# 4. TEST SCRIPTS
print_status "Creating test scripts..."

# Simple emotion tester
cat > "$PACKAGE_DIR/emotion_display_ws/src/emotion_display/test/test_emotions.py" << 'EOF'
#!/usr/bin/env python3
import rospy, sys, time
from std_msgs.msg import String

def main():
    rospy.init_node('test_emotions')
    pub = rospy.Publisher('/emotion_detection', String, queue_size=10)
    emotions = ["happy", "sad", "angry", "surprised", "joy", "fear", "normal"]
    
    print("🧪 JETSON EMOTION TESTER")
    if len(sys.argv) > 1:
        emotion = sys.argv[1].lower()
        if emotion in emotions:
            print(f"📤 Testing: {emotion}")
            msg = String(data=emotion)
            for _ in range(5): pub.publish(msg); time.sleep(0.1)
            print("✅ Sent!")
        else: print(f"❌ Use: {emotions}")
    else:
        print("🔄 Testing all emotions...")
        for emotion in emotions:
            print(f"📤 {emotion}")
            msg = String(data=emotion)
            for _ in range(10): pub.publish(msg); time.sleep(0.1)
            time.sleep(2)
        print("✅ Complete!")

if __name__ == '__main__': 
    try: main()
    except rospy.ROSInterruptException: pass
EOF

chmod +x "$PACKAGE_DIR/emotion_display_ws/src/emotion_display/test/test_emotions.py"

# Quick system test
cat > "$PACKAGE_DIR/emotion_display_ws/src/emotion_display/test/quick_test.sh" << 'EOF'
#!/bin/bash
echo "🧪 QUICK SYSTEM TEST"
LOCAL_IP=$(python3 -c "import socket; s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.connect(('8.8.8.8',80)); print(s.getsockname()[0]); s.close()")
echo "🌐 Testing: http://$LOCAL_IP:8000"

# Test endpoints
curl -s "http://$LOCAL_IP:8000/status" >/dev/null && echo "✅ Status OK" || echo "❌ Status FAIL"
curl -s "http://$LOCAL_IP:8000/emotion" >/dev/null && echo "✅ Emotion OK" || echo "❌ Emotion FAIL"

echo "📱 Tablet URL: http://$LOCAL_IP:8000"
echo "🎭 Test: rostopic pub /emotion_detection std_msgs/String \"data: 'happy'\""
EOF

chmod +x "$PACKAGE_DIR/emotion_display_ws/src/emotion_display/test/quick_test.sh"
print_success "Test scripts created"

# 5. INTEGRATION EXAMPLES
print_status "Creating integration examples..."

# OpenCV integration example
cat > "$PACKAGE_DIR/emotion_display_ws/src/emotion_display/examples/opencv_integration.py" << 'EOF'
#!/usr/bin/env python3
"""🎥 OPENCV EMOTION INTEGRATION EXAMPLE"""

import rospy, cv2, time
from std_msgs.msg import String

class OpenCVEmotionIntegration:
    def __init__(self):
        rospy.init_node('opencv_emotion_example')
        self.emotion_pub = rospy.Publisher('/emotion_detection', String, queue_size=1)
        self.cap = cv2.VideoCapture(0)
        print("🎥 OpenCV Emotion Integration Example")
        print("📹 Press 'h' for happy, 's' for sad, 'a' for angry, 'q' to quit")
        
    def run(self):
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret: continue
            
            # Display camera feed
            cv2.putText(frame, "Press: h=happy, s=sad, a=angry, q=quit", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow('Emotion Detection', frame)
            
            # Handle key presses
            key = cv2.waitKey(1) & 0xFF
            emotion = None
            if key == ord('h'): emotion = "happy"
            elif key == ord('s'): emotion = "sad" 
            elif key == ord('a'): emotion = "angry"
            elif key == ord('n'): emotion = "normal"
            elif key == ord('q'): break
            
            # Send emotion to display
            if emotion:
                msg = String(data=emotion)
                self.emotion_pub.publish(msg)
                print(f"📤 Sent: {emotion}")
                
        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try: OpenCVEmotionIntegration().run()
    except (rospy.ROSInterruptException, KeyboardInterrupt): pass
EOF

# Simple API integration
cat > "$PACKAGE_DIR/emotion_display_ws/src/emotion_display/examples/api_integration.py" << 'EOF'
#!/usr/bin/env python3
"""🌐 API EMOTION INTEGRATION EXAMPLE"""

import rospy, time, random
from std_msgs.msg import String

class APIEmotionExample:
    def __init__(self):
        rospy.init_node('api_emotion_example')
        self.emotion_pub = rospy.Publisher('/emotion_detection', String, queue_size=1)
        print("🌐 API Emotion Integration Example")
        
    def detect_emotion_from_api(self):
        """Simulate emotion detection from external API"""
        # This is where you'd call your actual emotion detection API
        emotions = ["happy", "sad", "angry", "surprised", "joy"]
        return random.choice(emotions)
        
    def run(self):
        rate = rospy.Rate(0.1)  # Check every 10 seconds
        while not rospy.is_shutdown():
            # Simulate getting emotion from your API
            detected_emotion = self.detect_emotion_from_api()
            
            # Send to display
            msg = String(data=detected_emotion)
            self.emotion_pub.publish(msg)
            print(f"🎭 API detected: {detected_emotion}")
            
            rate.sleep()

if __name__ == '__main__':
    try: APIEmotionExample().run()
    except (rospy.ROSInterruptException, KeyboardInterrupt): pass
EOF

chmod +x "$PACKAGE_DIR/emotion_display_ws/src/emotion_display/examples/"*.py
print_success "Integration examples created"

# 6. DOCUMENTATION
print_status "Creating documentation..."

# Main README
cat > "$PACKAGE_DIR/README.md" << 'EOF'
# 🤖 JETSON NANO EMOTION DISPLAY SYSTEM

**Ultra-fast robot emotion display for Android tablets**

## 🚀 QUICK START

1. **Extract this package** on your Jetson Nano
2. **Run:** `./INSTALL_ON_JETSON.sh`
3. **Start:** `~/emotion_display_ws/start_system.sh`
4. **Connect tablet:** Open browser → `http://JETSON_IP:8000`

## 📱 USAGE

### Send Emotions:
```bash
rostopic pub /emotion_detection std_msgs/String "data: 'happy'"
```

### In Your Code:
```python
import rospy
from std_msgs.msg import String

pub = rospy.Publisher('/emotion_detection', String, queue_size=1)
msg = String(data="happy")
pub.publish(msg)
```

## 🎭 EMOTIONS
`happy`, `sad`, `angry`, `surprised`, `joy`, `fear`, `normal`

## 🧪 TESTING
- **Quick test:** `~/emotion_display_ws/src/emotion_display/test/quick_test.sh`
- **All emotions:** `python3 test_emotions.py`
- **Specific emotion:** `python3 test_emotions.py happy`

## ✨ FEATURES
- ⚡ 100ms response time
- ⏰ 60-second emotion display  
- 📱 Tablet-optimized interface
- 🔄 Auto-refresh
- 🎨 Beautiful graphics

**Ready to use!** 🎉
EOF

# Quick reference card
cat > "$PACKAGE_DIR/QUICK_REFERENCE.txt" << 'EOF'
🤖 JETSON NANO EMOTION DISPLAY - QUICK REFERENCE

🚀 INSTALL: ./INSTALL_ON_JETSON.sh
🎯 START: ~/emotion_display_ws/start_system.sh
📱 TABLET: http://JETSON_IP:8000

🎭 SEND EMOTIONS:
rostopic pub /emotion_detection std_msgs/String "data: 'happy'"
rostopic pub /emotion_detection std_msgs/String "data: 'sad'"

🧪 TEST:
~/emotion_display_ws/src/emotion_display/test/quick_test.sh

📊 STATUS: 
curl http://localhost:8000/status

🎨 EMOTIONS: happy, sad, angry, surprised, joy, fear, normal
⏰ DURATION: 60 seconds (then returns to normal)
⚡ RESPONSE: <100ms from topic to tablet
EOF

print_success "Documentation created"

# 7. TOOLS AND UTILITIES
print_status "Creating tools..."

# Transfer helper
cat > "$PACKAGE_DIR/tools/transfer_to_jetson.sh" << 'EOF'
#!/bin/bash
# 📡 TRANSFER TO JETSON NANO HELPER

echo "📡 TRANSFER TO JETSON NANO"
echo "This helps you transfer the package to your Jetson Nano"

if [ $# -eq 0 ]; then
    echo "Usage: $0 <jetson_ip_address>"
    echo "Example: $0 192.168.1.100"
    exit 1
fi

JETSON_IP=$1
PACKAGE=$(ls -t *.tar.gz | head -n1)

if [ -z "$PACKAGE" ]; then
    echo "❌ No package file found"
    exit 1
fi

echo "📦 Package: $PACKAGE"
echo "🎯 Jetson IP: $JETSON_IP"
echo "📡 Transferring..."

# Transfer package
scp "$PACKAGE" jetson@$JETSON_IP:~/

# Connect and extract
ssh jetson@$JETSON_IP << EOF
    echo "📦 Extracting package..."
    tar -xzf $PACKAGE
    cd jetson_emotion_display_complete
    echo "🚀 Ready to install! Run: ./INSTALL_ON_JETSON.sh"
EOF

echo "✅ Transfer complete!"
echo "🔗 SSH to Jetson: ssh jetson@$JETSON_IP"
echo "📁 Run: cd jetson_emotion_display_complete && ./INSTALL_ON_JETSON.sh"
EOF

chmod +x "$PACKAGE_DIR/tools/transfer_to_jetson.sh"
print_success "Tools created"

# 8. CREATE FINAL ARCHIVE
print_status "Creating final package archive..."
cd /tmp
tar -czf "$ARCHIVE_NAME" "$PACKAGE_NAME"

# Get package info
PACKAGE_SIZE=$(du -h "$ARCHIVE_NAME" | cut -f1)
PACKAGE_PATH="/tmp/$ARCHIVE_NAME"

print_success "Package created: $PACKAGE_PATH ($PACKAGE_SIZE)"

# 9. FINAL INSTRUCTIONS
echo ""
echo "🎉 COMPLETE JETSON NANO EMOTION DISPLAY PACKAGE READY!"
echo "=" * 60
echo "📦 Package: $PACKAGE_PATH"
echo "📊 Size: $PACKAGE_SIZE"
echo "📅 Created: $(date)"
echo ""
echo "🚀 DEPLOYMENT STEPS:"
echo "1. Transfer package to Jetson Nano:"
echo "   scp $ARCHIVE_NAME jetson@JETSON_IP:~/"
echo ""
echo "2. On Jetson Nano, extract and install:"
echo "   tar -xzf $ARCHIVE_NAME"
echo "   cd $PACKAGE_NAME"
echo "   ./INSTALL_ON_JETSON.sh"
echo ""
echo "3. Start the system:"
echo "   ~/emotion_display_ws/start_system.sh"
echo ""
echo "4. Connect tablet:"
echo "   - Same WiFi as Jetson Nano"
echo "   - Browser: http://JETSON_IP:8000"
echo ""
echo "🎭 SEND EMOTIONS:"
echo "   rostopic pub /emotion_detection std_msgs/String \"data: 'happy'\""
echo ""
echo "🧪 QUICK TEST:"
echo "   python3 ~/emotion_display_ws/src/emotion_display/test/test_emotions.py"
echo ""
echo "📖 READ: README.md and QUICK_REFERENCE.txt in package"
echo ""
echo "✅ YOUR ROBOT EMOTION SYSTEM IS READY FOR DEPLOYMENT!"
echo "=" * 60

# Cleanup
rm -rf "$PACKAGE_DIR"

echo ""
print_info "Package location: $PACKAGE_PATH"
print_info "Ready for transfer to Jetson Nano!"
