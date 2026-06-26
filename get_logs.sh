#!/bin/bash

# ===== Configuration =====
TEAM=9140
RIO_HOST="roborio-${TEAM}-frc.local"
RIO_USER="lvuser"
REMOTE_LOG_DIR="/home/lvuser/logs"

# Desktop path (works in Git Bash/MinGW)
DESKTOP="$HOME/Desktop"

# Timestamped destination folder
TIMESTAMP=$(date +"%Y-%m-%d_%H-%M-%S")
DEST="$DESKTOP/roborio_logs_$TIMESTAMP"

mkdir -p "$DEST"

echo "Copying logs from $RIO_HOST..."

scp -r "${RIO_USER}@${RIO_HOST}:${REMOTE_LOG_DIR}/*" "$DEST/"

if [ $? -eq 0 ]; then
    echo "Copy successful."
    echo "Deleting logs on roboRIO..."

    ssh "${RIO_USER}@${RIO_HOST}" "rm -rf ${REMOTE_LOG_DIR}/*"

    if [ $? -eq 0 ]; then
        echo "Remote logs deleted successfully."
    else
        echo "WARNING: Failed to delete remote logs."
    fi
else
    echo "Copy failed. Remote logs were NOT deleted."
    exit 1
fi

echo "Done!"
echo "Logs saved to:"
echo "$DEST"