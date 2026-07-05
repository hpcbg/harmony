# Robotik4 Dashboard

> **A reusable module of the [HARMONY demonstrator](../README.md).** HARMONY combines seven
> independently reusable ARISE modules; this is one of them. **Standalone**, it is a customisable,
> widget-based dashboard app for any service exposing an HTTP/FIWARE API. **In the HARMONY
> demonstrator**, it is the live web view of system state. See the [top-level README](../README.md)
> for how the modules fit together.

This repository contains Dashboard app for robotic applications.

The app is currently deployed at https://robotic-dashboard.web.app

This dashboard allows you to get and visualize the information from various services which are providing HTTP API. The dashboard supports multiple type of widgets for visualization of different types of data.

You can take a look at the provided sample configuration and import them from the `Settings -> Import Configuration button`. The file [./sample-configs/dashboard-pack-bottle-config.json](./sample-configs/dashboard-pack-bottle-config.json) contains the configuration from the experiments of the minimum viable product.

Depending on the browser you might need to get permission for local network access. Also, depending on the security of the API service you might have problems with HTTPS permissions or CORS related issues. If you run the Dashboard locally it should be fine.

To run the Dashboard locally you need to execute:

```sh
npm install
npm run dev
```

**NOTE:** This version uses vite which might not run on older vesion of Node. It is tested on Note v22.22.1.

You can run it also with the script [./run.sh](./run.sh).

## Deploy to Firebase Hosting

```sh
npm run build
firebase login
firebase deploy
```