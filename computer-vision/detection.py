# detection.py — Optional semantic detection (Lunabotics)
#
# Extension point for "boulder vs crater" or other labels if the competition
# requires them. The current pipeline treats both as obstacles for navigation;
# this module can add explicit classification (e.g. for scoring or strategy).
#
# Possible uses:
#   - Classify obstacle endpoints as boulder (above-ground) vs crater (below-ground).
#   - Emit detections (bounding boxes, labels) for logging or downstream tasks.
#
# For now, mapping outputs positive (boulder) and negative (crater) obstacles
# geometrically; no separate detector needed for basic navigate-around behavior.
