### Problems i came across:


#### Motors
- To counter the torque on the drone 2 clockwise and 2 counter-clockwise motors are required. Else the drone will sping around the yaw-axis.

#### Communication
- The Transmitter&Receiver regulary lose signal : 
  - The cause may be current surges/drops. This this can be solved by soldering a capacitor to the nRF24 modules VSS & ground pin.
  - Test with receiver_raw.ino works fine. Must be a problem in the flight controller code.

#### License

If you find software that doesn’t have a license, that generally means you have no permission from the creators of the software to use, modify, or share the software. Although a code host such as GitHub may allow you to view and fork the code, this does not imply that you are permitted to use, modify, or share the software for any purpose.

Your options:

Ask the maintainers nicely to add a license. Unless the software includes strong indications to the contrary, lack of a license is probably an oversight. If the software is hosted on a site like GitHub, open an issue requesting a license and include a link to this site, or if you’re bold and it’s fairly obvious what license is most appropriate, open a pull request to add a license.

Don’t use the software. Find or create an alternative that is under an open source license.

Negotiate a private license. Bring your lawyer.
