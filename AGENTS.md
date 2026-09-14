Environment Notes

 ### Static File Serving

 - Flask's ControllerServer.__init__ creates the app with static_folder="static" — static files are served only from
   the static/ directory, not from parent dirs like assets/.
 - All image assets must be placed under static/ (or static/images/) to be served.

 ### Vehicle Image Path Convention

 - vehicles.json image values must be referenced via /static/images/ in index.html.
 - The HTML generates: src="/static/images/${v.image}".
 - So v.image should be "vehicles/Rollback.jpeg" (no assets/ prefix), and the file must live at
   static/images/vehicles/Rollback.jpeg.
 - If the image path contains assets/, the /static/images/ prefix will produce a wrong path.

 ### Placeholder Fallback

 - index.html uses an onerror handler on the <img> tag:
   ```js
     onerror="this.parentElement.innerHTML='<span class=\\'img-placeholder\\'>🚗</span>'"
   ```
   This renders a "🚗" placeholder when an image URL returns 404.

 ### API Endpoints

 - GET /api/vehicles → JSON list of vehicles
 - GET /api/state → JSON state snapshot
 - POST /api/select/<vehicle_id> → select vehicle
 - POST /api/deselect → deselect vehicle
 - WS /ws → push state updates to connected clients (WebSocket via flask-sock)

 ### Development Setup

 - run-dev.py is a stub that patches serial_bridge.SerialBridge with a no-op fake for testing without hardware.
 - The AppState class (in state.py) handles select() and deselect() methods and exposes on_change callback for
   state-driven WebSocket pushes.