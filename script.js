// Fetch data from the backend
const updateData = async () => {
  try {
    const response = await fetch("http://localhost:3000/data");
    const data = await response.json();

    document.getElementById("timeSpent").textContent = data.timeSpent;
    document.getElementById("distanceTravelled").textContent = data.distanceTravelled;
    document.getElementById("co2Emitted").textContent = data.co2Emitted;
  } catch (error) {
    console.error("Error fetching data:", error);
  }
};

setInterval(updateData, 100);