#!/bin/bash

# TurtleBot3 CBF Controller - Quick Deployment Script
# This script helps deploy the package to a ROS 2 workspace

set -e

echo "🤖 TurtleBot3 CBF Controller - Deployment Script"
echo "================================================"

# Check if ROS 2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS 2 is not sourced. Please run:"
    echo "   source /opt/ros/humble/setup.bash"
    exit 1
fi

echo "✅ ROS 2 $ROS_DISTRO detected"

# Check for workspace
if [ -z "$1" ]; then
    WORKSPACE="$HOME/ros2_ws"
    echo "📁 No workspace specified, using default: $WORKSPACE"
else
    WORKSPACE="$1"
    echo "📁 Using workspace: $WORKSPACE"
fi

# Create workspace if it doesn't exist
if [ ! -d "$WORKSPACE" ]; then
    echo "📁 Creating workspace: $WORKSPACE"
    mkdir -p "$WORKSPACE/src"
fi

# Copy package to workspace
PACKAGE_DIR="$WORKSPACE/src/turtle_pkg"
echo "📦 Copying package to $PACKAGE_DIR"

if [ -d "$PACKAGE_DIR" ]; then
    echo "⚠️  Package already exists. Removing old version..."
    rm -rf "$PACKAGE_DIR"
fi

cp -r "$(dirname "$0")" "$PACKAGE_DIR"

# Remove deployment script from copied package
rm -f "$PACKAGE_DIR/deploy.sh"

cd "$WORKSPACE"

echo "🔧 Installing dependencies..."
rosdep update
rosdep install --from-paths src --ignore-src -r -y

echo "🔨 Building package..."
colcon build --packages-select turtle_pkg

echo "📝 Sourcing workspace..."
source install/setup.bash

echo ""
echo "🎉 Deployment complete!"
echo ""
echo "To use the package:"
echo "1. Source the workspace:"
echo "   source $WORKSPACE/install/setup.bash"
echo ""
echo "2. Set TurtleBot3 model:"
echo "   export TURTLEBOT3_MODEL=burger"
echo ""
echo "3. Launch the controller:"
echo "   ros2 launch turtle_pkg turtlebot_cbf.launch.py"
echo ""
echo "Or run in simulation:"
echo "   ros2 launch turtle_pkg simulation.launch.py"
echo ""
echo "📚 See README.md for detailed instructions"