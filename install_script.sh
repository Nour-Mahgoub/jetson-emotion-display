#!/bin/bash

# 🚀 JETSON NANO EMOTION DISPLAY SYSTEM - COMPLETE INSTALLER
# Run this script on your Jetson Nano to install everything!

echo "🤖 JETSON NANO EMOTION DISPLAY SYSTEM INSTALLER"
echo "=" * 60

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if running as root
if [[ $EUID -eq 0 ]]; then
   print_error "This script should not be run as root"
   exit 1
fi

print_status "Starting installation..."

# 1. CREATE WORKSPACE STRUCTURE
print_status "Creating workspace structure..."
WORKSPACE_DIR="$HOME/emotion_display_ws"
SRC_DIR="$WORKSPACE_DIR/src/emotion_display"

if [ -d "$WORKSPACE_DIR" ]; then
    print_warning "Workspace already exists. Backing up..."
    mv "$WORKSPACE_DIR" "$WORKSPACE_DIR.backup.$(date +%Y%m%d_%H%M%S)"
fi

mkdir -p "$SRC_DIR"
mkdir -p "$SRC_DIR/scripts"
mkdir -p "$SRC_DIR/images"
mkdir -p "$SRC_DIR/launch"
mkdir -p "$SRC_DIR/test"

print_success "Workspace structure created"



# 3. INSTALL DEPENDENCIES
print_status "Installing dependencies..."
sudo apt-get install -y \
    python3-pil \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-compressed-image-transport \
    curl \
    wget

# Install Python packages
pip3 install --user pillow requests

print_success "Dependencies installed"

# 4. CREATE PACKAGE.XML
print_status "Creating ROS package configuration..."
cat > "$SRC_DIR/package.xml" << 'EOF'
<?xml version="1.0"?>
<package format="2">
  <name>emotion_display</name>
  <version>1.0.0</version>
  <description>Jetson Nano Emotion Display System for Android Tablets</description>

  <maintainer email="robot@example.com">Robot Team</maintainer>
  <license>MIT</license>

  <buildtool_depend>catkin</buildtool_depend>
  
  <depend>rospy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>cv_bridge</depend>
  <depend>image_transport</depend>

  <export>
  </export>
</package>
EOF

# 5. CREATE CMAKELISTS.TXT
cat > "$SRC_DIR/CMakeLists.txt" << 'EOF'
cmake_minimum_required(VERSION 3.0.2)
project(emotion_display)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
)

catkin_package()
EOF

print_success "ROS package configuration created"

# 6. CREATE MAIN EMOTION NODE
print_status "Creating emotion display node..."
cat > "$SRC_DIR/scripts/emotion_display_node.py" << 'EOF'
#!/usr/bin/env python3

"""
🤖 JETSON NANO EMOTION DISPLAY NODE
Fast, responsive emotion display for Android tablets
Subscribes to: /emotion_detection
Publishes to: HTTP server for tablet display
"""

import rospy
import cv2
import numpy as np
import base64
import json
import time
import threading
from std_msgs.msg import String
from http.server import HTTPServer, BaseHTTPRequestHandler
from PIL import Image, ImageDraw, ImageFont
import io

class EmotionDisplayNode:
    def __init__(self):
        rospy.init_node('emotion_display_node', anonymous=True)
        
        # Emotion state
        self.current_emotion = "normal"
        self.emotion_start_time = 0
        self.emotion_duration = 60  # 1 minute in seconds
        self.emotion_count = 0
        self.latest_image = None
        self.lock = threading.Lock()
        
        # Get parameters
        self.emotion_topic = rospy.get_param('~emotion_topic', '/emotion_detection')
        self.http_port = rospy.get_param('~http_port', 8000)
        self.tablet_ip = rospy.get_param('~tablet_ip', 'auto')
        
        print(f"🎭 Emotion Display Node Starting...")
        print(f"📡 Subscribing to: {self.emotion_topic}")
        print(f"🌐 HTTP Server port: {self.http_port}")
        
        # Subscribe to emotion detection
        self.emotion_sub = rospy.Subscriber(
            self.emotion_topic, 
            String, 
            self.emotion_callback, 
            queue_size=1
        )
        
        # Create initial normal image
        self.update_emotion_image()
        
        # Start HTTP server in separate thread
        self.start_http_server()
        
        print("✅ Emotion Display Node Ready!")
        
    def emotion_callback(self, msg):
        """Handle incoming emotion detection"""
        emotion = msg.data.lower().strip()
        current_time = time.time()
        
        with self.lock:
            # Check if this is a new emotion or update
            if emotion != self.current_emotion:
                self.current_emotion = emotion
                self.emotion_start_time = current_time
                self.emotion_count += 1
                
                print(f"🎭 New emotion detected: {emotion} (#{self.emotion_count})")
                
                # Generate new emotion image immediately
                self.update_emotion_image()
                
    def update_emotion_image(self):
        """Generate emotion image based on current state"""
        current_time = time.time()
        
        # Check if emotion should expire (back to normal after 1 minute)
        if (self.current_emotion != "normal" and 
            current_time - self.emotion_start_time > self.emotion_duration):
            self.current_emotion = "normal"
            print("⏰ Emotion expired, returning to normal")
        
        # Generate image
        image = self.create_emotion_image(self.current_emotion)
        
        # Convert to base64 for web display
        with self.lock:
            self.latest_image = self.image_to_base64(image)
            
    def create_emotion_image(self, emotion):
        """Create emotion image using PIL for better quality"""
        # Create high-quality image
        width, height = 800, 600
        
        # Emotion color schemes and emojis
        emotion_config = {
            "happy": {"color": (50, 205, 50), "emoji": "😊", "text": "HAPPY"},
            "sad": {"color": (30, 144, 255), "emoji": "😢", "text": "SAD"},
            "angry": {"color": (220, 20, 60), "emoji": "😠", "text": "ANGRY"},
            "surprised": {"color": (255, 215, 0), "emoji": "😲", "text": "SURPRISED"},
            "fear": {"color": (138, 43, 226), "emoji": "😨", "text": "FEAR"},
            "disgust": {"color": (154, 205, 50), "emoji": "🤢", "text": "DISGUST"},
            "joy": {"color": (255, 69, 0), "emoji": "😄", "text": "JOY"},
            "normal": {"color": (70, 70, 70), "emoji": "🤖", "text": "NORMAL"}
        }
        
        config = emotion_config.get(emotion, emotion_config["normal"])
        
        # Create PIL image
        img = Image.new('RGB', (width, height), color=config["color"])
        draw = ImageDraw.Draw(img)
        
        # Try to load a font, fallback to default
        try:
            font_large = ImageFont.truetype("/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf", 80)
            font_emoji = ImageFont.truetype("/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf", 200)
            font_small = ImageFont.truetype("/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf", 30)
        except:
            font_large = ImageFont.load_default()
            font_emoji = ImageFont.load_default()
            font_small = ImageFont.load_default()
        
        # Draw emoji (center top)
        emoji_text = config["emoji"]
        emoji_bbox = draw.textbbox((0, 0), emoji_text, font=font_emoji)
        emoji_width = emoji_bbox[2] - emoji_bbox[0]
        emoji_x = (width - emoji_width) // 2
        draw.text((emoji_x, 50), emoji_text, fill="white", font=font_emoji)
        
        # Draw emotion text (center)
        text = config["text"]
        text_bbox = draw.textbbox((0, 0), text, font=font_large)
        text_width = text_bbox[2] - text_bbox[0]
        text_x = (width - text_width) // 2
        draw.text((text_x, 350), text, fill="white", font=font_large)
        
        # Draw timestamp and info
        timestamp = time.strftime("%H:%M:%S")
        info_text = f"Time: {timestamp} | Count: {self.emotion_count}"
        draw.text((20, height - 50), info_text, fill="white", font=font_small)
        
        # Add subtle border
        draw.rectangle([(0, 0), (width-1, height-1)], outline="white", width=3)
        
        return img
    
    def image_to_base64(self, pil_image):
        """Convert PIL image to base64 string"""
        buffer = io.BytesIO()
        pil_image.save(buffer, format='JPEG', quality=95)
        img_str = base64.b64encode(buffer.getvalue()).decode()
        return f"data:image/jpeg;base64,{img_str}"
    
    def start_http_server(self):
        """Start HTTP server for tablet communication"""
        handler = type('EmotionHTTPHandler', (BaseHTTPRequestHandler,), {
            'emotion_node': self,
            'do_GET': self.handle_http_request,
            'do_OPTIONS': self.handle_options,
            'log_message': lambda self, format, *args: None  # Suppress logs
        })
        
        try:
            server = HTTPServer(('0.0.0.0', self.http_port), handler)
            server_thread = threading.Thread(target=server.serve_forever)
            server_thread.daemon = True
            server_thread.start()
            
            # Get local IP
            import socket
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            local_ip = s.getsockname()[0]
            s.close()
            
            print(f"🌐 HTTP Server started: http://{local_ip}:{self.http_port}")
            print(f"📱 Tablet URL: http://{local_ip}:{self.http_port}")
            
        except Exception as e:
            print(f"❌ HTTP Server error: {e}")
    
    def handle_options(self, request):
        """Handle CORS preflight requests"""
        request.send_response(200)
        request.send_header('Access-Control-Allow-Origin', '*')
        request.send_header('Access-Control-Allow-Methods', 'GET, OPTIONS')
        request.send_header('Access-Control-Allow-Headers', '*')
        request.end_headers()
    
    def handle_http_request(self, request):
        """Handle HTTP requests from tablet"""
        try:
            # CORS headers
            request.send_response(200)
            request.send_header('Access-Control-Allow-Origin', '*')
            request.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            request.send_header('Pragma', 'no-cache')
            request.send_header('Expires', '0')
            
            if request.path == '/emotion' or request.path.startswith('/emotion?'):
                # Serve emotion data
                request.send_header('Content-Type', 'application/json')
                request.end_headers()
                
                current_time = time.time()
                time_remaining = max(0, self.emotion_duration - (current_time - self.emotion_start_time))
                
                with self.lock:
                    response = {
                        'ok': True,
                        'image': self.latest_image,
                        'emotion': self.current_emotion,
                        'count': self.emotion_count,
                        'timestamp': current_time,
                        'time_remaining': time_remaining,
                        'duration': self.emotion_duration
                    }
                
                request.wfile.write(json.dumps(response).encode('utf-8'))
                
            elif request.path == '/status':
                # Serve status information
                request.send_header('Content-Type', 'application/json')
                request.end_headers()
                
                status = {
                    'node_active': True,
                    'current_emotion': self.current_emotion,
                    'emotion_count': self.emotion_count,
                    'ros_topic': self.emotion_topic,
                    'uptime': time.time() - rospy.get_time()
                }
                
                request.wfile.write(json.dumps(status).encode('utf-8'))
                
            else:
                # Serve main HTML page
                request.send_header('Content-Type', 'text/html')
                request.end_headers()
                request.wfile.write(self.get_html_page().encode('utf-8'))
                
        except Exception as e:
            print(f"HTTP Request error: {e}")
            request.send_error(500)
    
    def get_html_page(self):
        """Generate HTML page for tablet display"""
        return '''
<!DOCTYPE html>
<html>
<head>
    <title>🤖 Robot Emotions</title>
    <meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
    <style>
        body {
            margin: 0;
            padding: 0;
            background: #000;
            color: #fff;
            font-family: 'Arial', sans-serif;
            overflow: hidden;
            height: 100vh;
        }
        
        #emotion-image {
            width: 100vw;
            height: 100vh;
            object-fit: contain;
            display: block;
        }
        
        #status-bar {
            position: fixed;
            top: 10px;
            left: 10px;
            background: rgba(0,0,0,0.8);
            color: white;
            padding: 8px 15px;
            border-radius: 15px;
            font-size: 14px;
            backdrop-filter: blur(10px);
            z-index: 1000;
            border: 1px solid rgba(255,255,255,0.3);
        }
        
        #countdown {
            position: fixed;
            top: 10px;
            right: 10px;
            background: rgba(255,69,0,0.9);
            color: white;
            padding: 8px 15px;
            border-radius: 15px;
            font-size: 14px;
            font-weight: bold;
            z-index: 1000;
            display: none;
        }
        
        .pulse {
            animation: pulse 1s ease-in-out infinite;
        }
        
        @keyframes pulse {
            0% { opacity: 0.8; }
            50% { opacity: 1; }
            100% { opacity: 0.8; }
        }
    </style>
</head>
<body>
    <div id="status-bar">🤖 Ready</div>
    <div id="countdown"></div>
    <img id="emotion-image" src="" alt="Robot Emotion">

    <script>
        let emotionImage = document.getElementById('emotion-image');
        let statusBar = document.getElementById('status-bar');
        let countdown = document.getElementById('countdown');
        let lastEmotionCount = 0;
        
        function updateDisplay() {
            fetch('/emotion?t=' + Date.now())
                .then(response => response.json())
                .then(data => {
                    if (data.ok && data.image) {
                        emotionImage.src = data.image;
                        
                        // Update status
                        statusBar.textContent = `🎭 ${data.emotion.toUpperCase()} (#${data.count})`;
                        
                        // Show countdown if not normal emotion
                        if (data.emotion !== 'normal' && data.time_remaining > 0) {
                            countdown.style.display = 'block';
                            countdown.textContent = `⏰ ${Math.ceil(data.time_remaining)}s`;
                            
                            if (data.time_remaining < 10) {
                                countdown.classList.add('pulse');
                            } else {
                                countdown.classList.remove('pulse');
                            }
                        } else {
                            countdown.style.display = 'none';
                        }
                        
                        // Flash effect for new emotions
                        if (data.count > lastEmotionCount) {
                            document.body.style.filter = 'brightness(1.5)';
                            setTimeout(() => {
                                document.body.style.filter = 'brightness(1)';
                            }, 200);
                            lastEmotionCount = data.count;
                        }
                    }
                })
                .catch(error => {
                    statusBar.textContent = '❌ Connection Error';
                    console.error('Error:', error);
                });
        }
        
        // Update every 100ms for fast response
        setInterval(updateDisplay, 100);
        updateDisplay();
        
        // Prevent sleep/screensaver
        let wakeLock = null;
        if ('wakeLock' in navigator) {
            navigator.wakeLock.request('screen').then(wl => wakeLock = wl);
        }
    </script>
</body>
</html>
        '''
    
    def run(self):
        """Main run loop"""
        rate = rospy.Rate(10)  # 10Hz for emotion updates
        
        while not rospy.is_shutdown():
            # Update emotion image periodically
            self.update_emotion_image()
            rate.sleep()

if __name__ == '__main__':
    try:
        node = EmotionDisplayNode()
        node.run()
    except rospy.ROSInterruptException:
        print("🛑 Emotion Display Node stopped")
    except KeyboardInterrupt:
        print("🛑 Emotion Display Node interrupted")
EOF

chmod +x "$SRC_DIR/scripts/emotion_display_node.py"
print_success "Main emotion node created"

# 7. CREATE LAUNCH FILE
print_status "Creating launch files..."
cat > "$SRC_DIR/launch/emotion_display.launch" << 'EOF'
<launch>
    <!-- Emotion Display System Launch File -->
    
    <arg name="emotion_topic" default="/emotion_detection" />
    <arg name="http_port" default="8000" />
    
    <node pkg="emotion_display" 
          type="emotion_display_node.py" 
          name="emotion_display_node" 
          output="screen">
        <param name="emotion_topic" value="$(arg emotion_topic)" />
        <param name="http_port" value="$(arg http_port)" />
    </node>
    
</launch>
EOF

print_success "Launch file created"

# 8. CREATE TEST SCRIPTS
print_status "Creating test scripts..."

# Test emotion publisher
cat > "$SRC_DIR/test/test_emotion_publisher.py" << 'EOF'
#!/usr/bin/env python3

"""
🧪 TEST EMOTION PUBLISHER
Publishes test emotions to verify the system
"""

import rospy
from std_msgs.msg import String
import time
import sys

def test_emotions():
    rospy.init_node('test_emotion_publisher', anonymous=True)
    pub = rospy.Publisher('/emotion_detection', String, queue_size=10)
    
    emotions = ["happy", "sad", "angry", "surprised", "joy", "fear", "normal"]
    
    print("🧪 TEST EMOTION PUBLISHER")
    print("📡 Publishing to: /emotion_detection")
    print("🎭 Available emotions:", emotions)
    print("=" * 50)
    
    rate = rospy.Rate(1)  # 1Hz
    
    if len(sys.argv) > 1:
        # Test specific emotion
        emotion = sys.argv[1].lower()
        if emotion in emotions:
            print(f"📤 Testing emotion: {emotion}")
            msg = String()
            msg.data = emotion
            pub.publish(msg)
            print("✅ Emotion sent!")
        else:
            print(f"❌ Invalid emotion: {emotion}")
            print(f"Valid emotions: {emotions}")
    else:
        # Cycle through all emotions
        print("🔄 Cycling through all emotions (5 seconds each)...")
        for emotion in emotions:
            if rospy.is_shutdown():
                break
                
            print(f"📤 Sending: {emotion}")
            msg = String()
            msg.data = emotion
            
            # Send emotion multiple times to ensure delivery
            for i in range(5):
                pub.publish(msg)
                rate.sleep()
            
            print(f"✅ {emotion} sent for 5 seconds")
            
        print("🏁 Test completed!")

if __name__ == '__main__':
    try:
        test_emotions()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\n🛑 Test stopped")
EOF

chmod +x "$SRC_DIR/test/test_emotion_publisher.py"

# Network test script
cat > "$SRC_DIR/test/test_network.py" << 'EOF'
#!/usr/bin/env python3

"""
🌐 NETWORK TEST SCRIPT
Tests network connectivity for tablet communication
"""

import socket
import requests
import json
import time

def get_local_ip():
    """Get local IP address"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except:
        return "localhost"

def test_network():
    print("🌐 NETWORK CONNECTIVITY TEST")
    print("=" * 40)
    
    # Get IP
    local_ip = get_local_ip()
    print(f"🖥️  Local IP: {local_ip}")
    
    # Test HTTP server
    test_url = f"http://{local_ip}:8000"
    print(f"🔗 Testing URL: {test_url}")
    
    try:
        response = requests.get(f"{test_url}/status", timeout=5)
        if response.status_code == 200:
            data = response.json()
            print("✅ HTTP Server: WORKING")
            print(f"📊 Status: {json.dumps(data, indent=2)}")
        else:
            print(f"❌ HTTP Server: ERROR {response.status_code}")
    except requests.exceptions.ConnectionError:
        print("❌ HTTP Server: NOT RUNNING")
        print("💡 Start the emotion display node first!")
    except Exception as e:
        print(f"❌ HTTP Server: ERROR - {e}")
    
    # Test emotion endpoint
    try:
        response = requests.get(f"{test_url}/emotion", timeout=5)
        if response.status_code == 200:
            data = response.json()
            print("✅ Emotion API: WORKING")
            print(f"🎭 Current emotion: {data.get('emotion', 'unknown')}")
        else:
            print(f"❌ Emotion API: ERROR {response.status_code}")
    except Exception as e:
        print(f"❌ Emotion API: ERROR - {e}")
    
    print("=" * 40)
    print(f"📱 Tablet URLs to test:")
    print(f"   Main page: {test_url}")
    print(f"   Status: {test_url}/status")
    print(f"   Emotion API: {test_url}/emotion")

if __name__ == '__main__':
    test_network()
EOF

chmod +x "$SRC_DIR/test/test_network.py"

# Complete system test
cat > "$SRC_DIR/test/run_complete_test.sh" << 'EOF'
#!/bin/bash

# 🧪 COMPLETE SYSTEM TEST

echo "🧪 COMPLETE EMOTION DISPLAY SYSTEM TEST"
echo "=" * 50

# Check if ROS is running
if ! pgrep -x "roscore" > /dev/null; then
    echo "❌ ROS Core not running!"
    echo "💡 Start roscore first: roscore"
    exit 1
fi

echo "✅ ROS Core is running"

# Check if emotion node is running
if ! rostopic list | grep -q "/emotion_detection"; then
    echo "⚠️  Emotion detection topic not found"
    echo "💡 This is normal if no emotion detector is running yet"
fi

# Test network
echo "🌐 Testing network connectivity..."
python3 test_network.py

echo ""
echo "🎭 Testing emotion publishing..."
echo "   This will send test emotions for 30 seconds..."

# Start emotion node in background
echo "🚀 Starting emotion display node..."
roslaunch emotion_display emotion_display.launch &
NODE_PID=$!

# Wait for node to start
sleep 3

# Test emotions
echo "📤 Sending test emotions..."
python3 test_emotion_publisher.py &
TEST_PID=$!

# Wait for test
sleep 35

# Cleanup
echo "🧹 Cleaning up..."
kill $TEST_PID 2>/dev/null
kill $NODE_PID 2>/dev/null

echo "✅ Test completed!"
echo "📱 Check tablet at: http://$(python3 -c 'import socket; s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.connect(("8.8.8.8",80)); print(s.getsockname()[0]); s.close()'):8000"
EOF

chmod +x "$SRC_DIR/test/run_complete_test.sh"

print_success "Test scripts created"

# 9. BUILD WORKSPACE
print_status "Building ROS workspace..."
cd "$WORKSPACE_DIR"
catkin_make

if [ $? -eq 0 ]; then
    print_success "Workspace built successfully"
else
    print_error "Workspace build failed"
    exit 1
fi

# 10. CREATE STARTUP SCRIPT
print_status "Creating startup scripts..."
cat > "$WORKSPACE_DIR/start_emotion_system.sh" << 'EOF'
#!/bin/bash

# 🚀 EMOTION DISPLAY SYSTEM STARTUP SCRIPT

echo "🤖 Starting Emotion Display System..."

# Source ROS environment
source /opt/ros/noetic/setup.bash
source ~/emotion_display_ws/devel/setup.bash

# Check if roscore is running
if ! pgrep -x "roscore" > /dev/null; then
    echo "🚀 Starting ROS Core..."
    roscore &
    sleep 3
fi

# Start emotion display system
echo "🎭 Starting Emotion Display Node..."
roslaunch emotion_display emotion_display.launch

echo "✅ Emotion Display System started!"
EOF

chmod +x "$WORKSPACE_DIR/start_emotion_system.sh"

# 11. CREATE DESKTOP SHORTCUT
cat > "$HOME/Desktop/Emotion_Display.desktop" << EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=Robot Emotion Display
Comment=Start Robot Emotion Display System
Exec=$WORKSPACE_DIR/start_emotion_system.sh
Icon=$SRC_DIR/images/robot_icon.png
Terminal=true
Categories=Application;
EOF

chmod +x "$HOME/Desktop/Emotion_Display.desktop"

print_success "Startup scripts created"

# 12. ADD TO BASHRC
print_status "Adding to bashrc..."
echo "" >> ~/.bashrc
echo "# Robot Emotion Display System" >> ~/.bashrc
echo "source $WORKSPACE_DIR/devel/setup.bash" >> ~/.bashrc

print_success "Environment configured"

# 13. FINAL SETUP
print_status "Final setup..."

# Get local IP for instructions
LOCAL_IP=$(python3 -c "import socket; s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.connect(('8.8.8.8',80)); print(s.getsockname()[0]); s.close()" 2>/dev/null || echo "localhost")

# Create quick reference
cat > "$WORKSPACE_DIR/QUICK_REFERENCE.txt" << EOF
🤖 JETSON NANO EMOTION DISPLAY SYSTEM - QUICK REFERENCE

📁 WORKSPACE: $WORKSPACE_DIR

🚀 START SYSTEM:
   $WORKSPACE_DIR/start_emotion_system.sh

🧪 TEST SYSTEM:
   cd $WORKSPACE_DIR/src/emotion_display/test
   ./run_complete_test.sh

📱 TABLET URLS:
   Main Display: http://$LOCAL_IP:8000
   Status: http://$LOCAL_IP:8000/status
   API: http://$LOCAL_IP:8000/emotion

🎭 SEND TEST EMOTIONS:
   rostopic pub /emotion_detection std_msgs/String "data: 'happy'"
   rostopic pub /emotion_detection std_msgs/String "data: 'sad'"

📊 CHECK STATUS:
   rostopic echo /emotion_detection
   curl http://$LOCAL_IP:8000/status

🔧 LOGS:
   rosnode list
   rostopic list
   rostopic info /emotion_detection
EOF

print_success "Installation completed!"

echo ""
echo "🎉 INSTALLATION COMPLETE!"
echo "=" * 50
echo "📁 Workspace: $WORKSPACE_DIR"
echo "🌐 Your IP: $LOCAL_IP"
echo "📱 Tablet URL: http://$LOCAL_IP:8000"
echo ""
echo "🚀 TO START:"
echo "   $WORKSPACE_DIR/start_emotion_system.sh"
echo ""
echo "🧪 TO TEST:"
echo "   cd $WORKSPACE_DIR/src/emotion_display/test"
echo "   ./run_complete_test.sh"
echo ""
echo "📖 Check README.md for detailed instructions!"
echo "=" * 50
