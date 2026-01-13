# Campus Meeting Point App (C++)

A C++ route-planning application that uses **OpenStreetMap** data to help two users find a convenient meeting point on the UIC campus and compute the **shortest walking paths** using **Dijkstra’s algorithm**.

---

## What this project does

- Parses OpenStreetMap `.osm` data for the UIC area.
- Builds a graph of intersections/ways from the map data.
- Lets two users select starting locations (campus buildings/POIs).
- Computes shortest paths and reports a meeting point and routes.

---

## Tech / Concepts

- **Language:** C++
- **Map data:** OpenStreetMap `.osm`
- **XML parsing:** TinyXML-2 (included in the repo as `tinyxml2.*`)
- **Algorithms / DS:** Graph representation + **Dijkstra shortest path**
- **Build:** `make`

---

## File overview

Key files in this repo include: `application.cpp`, `osm.cpp/.h`, `graph.h`, `dist.cpp/.h`, `tinyxml2.cpp/.h`, `makefile`, and map data files like `uic.osm` / `map.osm`.

---

## Build & Run (Linux/macOS)

1. **Clone**
   ```bash
   git clone https://github.com/TheRoadSurgeon/Campus-Meeting-Point-App.git
   cd Campus-Meeting-Point-App
   ```

2. **Build**
   ```bash
   make
   ```

3. **Run**
   ```bash
   ./application
   ```

> If your output binary name differs, run `ls` after `make` and execute the generated program name.

---

## Usage

Typical flow:
1. Launch the program
2. Choose a campus map file (or the default one included)
3. Enter two starting locations (building names / POIs depending on the dataset)
4. The program outputs the meeting point and shortest-path distances/routes

---

## Testing

This repo includes `testing.cpp`. Depending on your build setup, you can compile and run tests separately or configure a dedicated test target.

---

## Map data and attribution

This project uses OpenStreetMap data (`.osm`). If you keep `.osm` files in the repo, include attribution and comply with OpenStreetMap’s license requirements.

- OpenStreetMap contributors: https://www.openstreetmap.org/copyright

---

## Future improvements

- Add clearer CLI prompts and input validation for building name lookup
- Print the actual path (node list) or export to a simple CSV/GeoJSON for visualization
- Add unit tests for graph building and shortest-path logic
- Add a small demo GIF/screenshot in the README

---