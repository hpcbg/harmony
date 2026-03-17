# Robotik4 Dashboard

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