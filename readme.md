# Inlämningsuppgift 2 för AI - VT22

## Grupp 5
Simon Eklundh  
Max Nyström  
Marcus Wallén

Projektlänk: https://github.com/sfkpmr/AI

## Requirements:
- Processing v4
- Minim ddf library
- AI for 2D Games
## Testing:
Install Processing v4. Install the libraries through Sketch > Import library > Add library. Then hit run.

## Testing tips:
Increase the numbers on lines 147 and 148 in class Tank for faster moving tanks. Remember to increase the max force when increasing max speed to avoid jojo effects on the tank movement. Max speed 50 and max force 5 have been used during testing.

## Notes:

The top tank is broken due to path finding errors in the bottom tank leading to it not knowing any node around it that it can go to. This could be resolved by giving it some sort of path finding but due to time restraints, we have chosen to leave it broken for now. It's possible to sidestep this issue by using the higher speeds under testing tips.

Adjust volume before running, shooting tanks can be very loud.
