# Inlämningsuppgift 1 för AI - VT22

## Grupp 5
Simon Eklundh  
Max Nyström  
Marcus Wallén

Projektlänk: https://github.com/sfkpmr/AI

## Requirements:
Requires Processing v4.
Requires the Minim ddf library

## Testing:
Install Processing v4. Install the Minim library through Sketch > Import library > Add library. Then hit run.

## Notes:
Tank 3 works, the others will not due to hardcoded changes in GUI functions, lines 53-65.

The tank will only find tanks twice, then it gets stuck in the bottom left corner because of the poor implementation of the looking part in the wayfinding algorithm used to find new places to go to.
