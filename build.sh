#!/bin/bash
# Build script for ROS 2 + Gazebo + PX4 Docker images

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if Docker is running
if ! docker info > /dev/null 2>&1; then
    print_error "Docker is not running. Please start Docker and try again."
    exit 1
fi

# Function to build base image
build_base() {
    print_info "Building image (Ubuntu 24.04 LTS + ROS 2 Jazzy + Gazebo Harmonic)"
    docker build -f ros2-jazzy-gazebo-harmonic.Dockerfile -t erdemuysalx/ros2-jazzy-gazebo-harmonic:latest .
    print_info "Image built successfully!"
}

# Function to build full image
build_full() {
    print_info "Building image (Ubuntu 24.04 LTS + ROS 2 Jazzy + Gazebo Harmonic + PX4 Autopilot + MAVROS + NoVNC)"
    
    # Check if base image exists
    if ! docker images erdemuysalx/ros2-jazzy-gazebo-harmonic:latest | grep -q erdemuysalx/ros2-jazzy-gazebo-harmonic; then
        print_warn "Base image not found. Building base image first..."
        build_base
    fi
    
    docker build -f px4-sitl.Dockerfile -t erdemuysalx/px4-sitl:latest .
    print_info "Image built successfully!"
}

# Function to build all images
build_all() {
    build_base
    build_full
}

# Function to show usage
show_usage() {
    cat << EOF
Usage: $0 [COMMAND]

Commands:
    --base    Build only the base image (ROS 2 + Gazebo)
    --full    Build full image (includes PX4 + VNC)
    --all     Build all images (base + full)
    --help    Show this help message

Examples:
    $0 --base          # Build base image only
    $0 --full          # Build full image (builds base if needed)
    $0 --all           # Build all images

After building, use docker-compose to run:
    docker-compose up           # Interactive mode
    docker-compose up -d        # Detached mode
EOF
}

# Main script
case "$1" in
    base|--base|-b)
        build_base
        ;;
    full|--full|-f)
        build_full
        ;;
    all|--all|-a)
        build_all
        ;;
    help|--help|-h)
        show_usage
        ;;
    *)
        print_error "Invalid command: $1"
        echo ""
        show_usage
        exit 1
        ;;
esac