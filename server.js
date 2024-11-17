const express = require("express");
const bodyParser = require("body-parser");
const cors = require("cors");

const app = express();
const PORT = 3000;

app.use(bodyParser.json());
app.use(cors());

let timeSpent = 0; // In minutes
let distanceTravelled = 0; // In liters
let co2Emitted = 0; // In kg

app.post("/update", (req, res) => {
  const { time, distance, co2 } = req.body;

  timeSpent = time;
  distanceTravelled = distance;
  co2Emitted = co2;

  console.log(`Received -> Time: ${time}, Distance: ${distance}, CO2: ${co2}`);
  res.sendStatus(200);
});

app.get("/data", (req, res) => {
  res.json({ timeSpent, distanceTravelled, co2Emitted });
});

app.listen(PORT, () => {
  console.log(`Server is running on http://localhost:${PORT}`);
});
