const BASE_HEADERS = {
  Accept: "application/json",
};

const FIWARE_HEADERS = {
  ...BASE_HEADERS,
  "Fiware-Service": "openiot",
  "Fiware-ServicePath": "/",
};

const delay = (ms) => new Promise((r) => setTimeout(r, ms));

const api = {
  async get(endpoint) {
    // FIWARE entities
    if (endpoint.includes("entities")) {
      const res = await fetch(endpoint, {
        method: "GET",
        headers: FIWARE_HEADERS,
      });

      if (!res.ok) {
        throw new Error(await res.text());
      }

      const data = await res.json();

      if (typeof data.value === "string" && data.value.includes("http")) {
        return { imageUrl: data.value };
      }

      return data;
    }

    // Camera stream
    if (endpoint.includes("camera")) {
      return {
        imageUrl: `${endpoint}?t=${Date.now()}`,
      };
    }

    // Mock delay
    await delay(300);

    // Mock endpoints
    if (endpoint.includes("/number")) {
      return { value: Math.floor(Math.random() * 100) };
    }

    if (endpoint.includes("/state")) {
      return { value: Math.random() > 0.5 };
    }

    if (
      endpoint.includes("/historyLineChart") ||
      endpoint.includes("/historyBarChart")
    ) {
      return { value: Math.floor(Math.random() * 100) };
    }

    if (endpoint.includes("/image")) {
      return {
        imageUrl: `https://picsum.photos/seed/${Math.random()}/800/600`,
      };
    }

    return { value: 0 };
  },

  async post(endpoint, data) {
    await delay(300);
    return { success: true, data };
  },

  async put(endpoint, data) {
    await delay(300);
    return { success: true, data };
  },

  async delete(endpoint) {
    await delay(300);
    return { success: true };
  },
};

export default api;
