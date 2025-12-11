// Helpers for reading/writing JSON to localStorage
export function loadJSON(key, fallback = null) {
  try {
    const stored = localStorage.getItem(key);
    if (!stored) return fallback;
    return JSON.parse(stored);
  } catch (err) {
    console.error("Failed to parse localStorage key:", key, err);
    return fallback;
  }
}

export function saveJSON(key, value) {
  try {
    localStorage.setItem(key, JSON.stringify(value));
  } catch (err) {
    console.error("Failed to save localStorage key:", key, err);
  }
}
