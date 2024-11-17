const express = require("express");
const bodyParser = require("body-parser");
const cors = require("cors");

const app = express();
const PORT = 3000;

app.use(bodyParser.json());
app.use(cors());

let timeSpent = 0; // In minutes
let gasBurned = 0; // In liters
let co2Emitted = 0; // In kg

app.post("/update", (req, res) => {
  const { time, gas, co2 } = req.body;

  timeSpent = time;
  gasBurned = gas;
  co2Emitted = co2;

  console.log(`Received -> Time: ${time}, Gas: ${gas}, CO2: ${co2}`);
  res.sendStatus(200);
});

app.get("/data", (req, res) => {
  res.json({ timeSpent, gasBurned, co2Emitted });
});

app.listen(PORT, () => {
  console.log(`Server is running on http://localhost:${PORT}`);
});
