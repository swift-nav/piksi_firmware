Piksi Firmware Release Notes
============================

Contents
--------

 * [v0.13](#v0.13)
 * [v0.12](#v0.12)
 * [v0.11](#v0.11)
 * [v0.10](#v0.10)
 * [v0.8](#v0.8)

v0.13 <a name="v0.13"></a>
-----

 * Fix a bug resulting in a stack overflow when satellite set changes
 * Fix a bug in the NMEA output when the longitude magnitude was >127
 * Simulator mode enable is now a setting rather than a button in the console
 * Numerous improvements to the development environment setup script including
   now using Ansible to provision the required tools

v0.12 <a name="v0.12"></a>
-----

 * Fixes a bug where a "disorder in the sdiffs" message is displayed and the
   RTK solution output stops
 * Fixes an issue where Piksi would stop outputting observations and solutions
   for a period of several minutes before resuming
 * Adds a "lock count" field to the observation message format, allowing the
   remote receiver to detect when a satellite has been reinitialized. This
   fixes a problem where the RTK solution would become large and erronious when
   one receiver is restarted or satellites are rapidly lost and re-gained
 * Fixes issue where console would freeze on start-up
 * Increases the precision of settings values, fixing an issue where the base
   station position would lose precision if the settings were saved and the
   Piksi was then reset
 * Added a "Reset to factory defaults" button to the console
 * Fixes the center on solution button in the console
 * Fixes the start date in RINEX file headers
 * Improves handling of the Piksi being unplugged in the console

v0.11 <a name="v0.11"></a>
-----

 * Add ‘low latency mode’ which provides very low latency baseline outputs
   irrespective of the radio communications delay with a small accuracy penalty
 * Fix a stack overflow bug that occurs when the number of satellites in common
   falls below 5
 * Improve the robustness of the firmware update procedure

v0.10 <a name="v0.10"></a>
-----

#### Core

 * FFT based acquisition core for ~100x increase in acquisition speed
 * Increased from 9 to 11 tracking channels
 * Long SPI transfers to the NAP now use DMA
 * Numerous improvements to the simulator to make it more closely match the
   non-simulated behaviour
 * Fix bug where settings were not always saved to flash
 * Fix numerous stability issues
 * Firmware now checks for compatible NAP version
 * Added monitoring of UART throughput and observation latency
 * New compacted observation format for increased throughput and flexible over
   the wire frame size
 * Refactored UART code to allow peripherals (e.g. Bluetooth or 3G modules) to
   claim a UART for exclusive use
 * Remove the old Float Kalman Filter with position states that was running in
   the background, now we exclusively run the new KF which only uses ambiguity
   states and constraint equations
 * Add FLL aiding to the carrier tracking loop PLL during the initial pull-in
   period
 * Add ability to periodically transmit a message indicating the surveyed
   location of a base station
 * Add support for external modules to the build system
 * Add mechanism to register NMEA dispatchers (used by the external iAP
   Bluetooth module to interface to Apple devices)
 * If we have received a message with the surveyed location of the base station
   then output "pseudo-absolute" NMEA messages

#### libswiftnav

 * Generate a fixed solution even if only a partial set of PRNs have had their
   integer ambiguity resolved
 * Add FLL-aided-PLL loop filter
 * Fixed bug where the reference satellite was not found
 * Numerous bug fixes and stability improvements

#### Console

 * Check for updated Piksi firmware, NAP firmware and console version
   on start-up and prompt the user to flash updated versions to the device
 * Allow the user to select a serial port using a GUI dialog if one is not
   specified on the command line
 * Binary builds of the console now available for Window and Mac OS X
 * Fixed a bug with the RINEX header format output
 * Numerous fixes to support a wider range of Python/Qt/Wx environments
 * Numerous cosmetic enhancements
 * Format of CSV log files improved for greater precision and more readable
   time format

v0.8 <a name="v0.8"></a>
----

 * First public beta release, as shipped with Kickstarter rewards and pre-order
   hardware
 * Basic RTK functionality

