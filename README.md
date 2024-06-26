![Artery V2X Simulation Framework](https://raw.githubusercontent.com/riebl/artery/master/logo.png)

Artery enables V2X simulations based on ETSI ITS-G5 protocols like GeoNetworking and BTP.
Single vehicles can be equipped with multiple ITS-G5 services through Artery's middleware, which also provides common Facilities for these services.

Artery started as an extension of the [Veins framework](http://veins.car2x.org) but can be used independently nowadays.
Please refer to its [documentation](http://veins.car2x.org/documentation) for details about Veins.

## INET 4 Port: TODO
* main project's CMakeLists.txt contains redundant parts for building with/without crowNet - should be cleaned up
* per-packet mode control is currently not implemented (see VanetMgmnt, etc.)
* frame capturing needs to be checked (see VanetReceiver.cc)
* VanetHcf: set mode individually for every frame not implemented (artery/inet/VanetHcf.cpp)

## Documentation

We have started to extend the available documentation about Artery and created a website for this purpose.
Please visit [artery.v2x-research.eu](http://artery.v2x-research.eu).
The [install instructions](http://artery.v2x-research.eu/install/) previously found in this README have also been moved to this website.
We welcome your contribution to this documentation effort just as to Artery itself.
If you want to build the website yourself see also [our mkdocs guide](http://artery.v2x-research.eu/mkdocs).