# Docker file for FIWARE server

This repository contains docker compose file for running FIWARE.

The server allows all CORS origins.

HTTPS is not provided, so it will not work if HTTPS is required by the browser. This will be solved in further updates by using self-signed certificates.

## Useful commands

- Start: `docker-compose up -d`
- Stop: `docker-compose down`
- Restart: `docker-compose restart`
- Check containers: `docker-compose ps
- Logs: `docker-compose logs -f orion`
- Remove and start over: `docker-compose down -v`
- Get version: `curl http://192.168.X.X:1026/version`
- Read value:
```
curl http://localhost:1026/v2/entities/test001 \
  -H 'Fiware-Service: openiot' \
  -H 'Fiware-ServicePath: /'
```

## Sample code for JavaScript
```js
fetch('http://192.168.X.X:1026/v2/entities', {
  headers: {
    'Fiware-Service': 'openiot',
    'Fiware-ServicePath': '/'
  }
})
.then(r => r.json())
.then(data => console.log(data))
.catch(err => console.error(err));
```