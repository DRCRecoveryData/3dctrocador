#!/bin/bash

# Create the 3DC folder if it doesn't exist
mkdir -p /usr/data/printer_data/config/3DC

# Copy the main macro file to the correct folder
cp troca_cor.cfg /usr/data/printer_data/config/3DC/troca_cor.cfg

# Set permissions to ensure proper execution
chmod 644 /usr/data/printer_data/config/3DC/troca_cor.cfg
