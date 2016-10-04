## How to install

You will need the following Arduino libraries :
- From the library manager : `Wifi101` `Adafruit_PN532`
- Manually installed : [`dream-babbling-arduino-framework`](https://github.com/fricher/dream-babbling-arduino-framework) [`mkr1000-hx8357d`](https://github.com/fricher/mkr1000-hx8357d)

Install create_ap from this repo : [`oblique/create_ap`](https://github.com/oblique/create_ap)

Then clone this repo in your catkin workspace :
```
git clone https://github.com/fricher/dream_babbling_modules_ros.git ~/catkin_ws/src/dream_babbling_modules
```

## How to run

- Start the access point `rosrun dream_babbling_modules start_ap.sh`
- Start the supervisor `rosrun dream_babbling_modules supervisor`

And that's all, now you can power on some modules and check that they are recognized by the supervisor.
