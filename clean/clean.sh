#!/usr/bin/env bash

echo "Cleaning generated files..."

# Remove generated maps
if [ -d "maps" ]; then
    echo "Removing maps/"
    rm -rf maps/*
fi

# Remove generated routes
if [ -d "routes" ]; then
    echo "Removing routes/"
    rm -rf routes/*
fi

# Remove experimental outputs
if [ -d "outputs" ]; then
    echo "Removing outputs/"
    rm -rf outputs/*
fi

# Remove Python cache
echo "Removing Python cache..."
find . -type d -name "__pycache__" -exec rm -rf {} +
find . -name "*.pyc" -delete
find . -name "*.pyo" -delete

# Remove temporary files
echo "Removing temporary files..."
find . -name "*.log" -delete
find . -name "*.tmp" -delete
find . -name "*.bak" -delete

echo "Cleanup complete."