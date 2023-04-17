const char UI_page[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>ACT 65: Car Follower</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
     .card {
        max-width: 400px;
        min-height: 250px;
        background: #02b875;
        padding: 30px;
        box-sizing: border-box;
        color: #FFF;
        margin: 20px;
        box-shadow: 0px 2px 18px -4px rgba(0,0,0,0.75);
    }
    .background-UI {
        background: #02b875;
        padding: 100px;
        color: #FFF;
        margin: 20px;
        box-shadow: 0px 2px 18px -4px rgba(0,0,0,0.75);
        text-align:center;
    }
  </style>
</head>
<body style="background-color:powderblue;">
    <div class="background-UI">
        <h2>Autonomous Car Tailer </h2><br>
        <h1><span id="Distance">Parked, No cars detected</span></h1><br>
    </div>
    <script>

setInterval(function() {
  // Call a function repetatively with 2 Second interval
  getData();
}, 500); //2000mSeconds update rate

function getData() {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("Distance").innerHTML =
      this.responseText;
    }
  };
  xhttp.open("GET", "readDistance", true);
  xhttp.send();
}
    </script>
</body>
</html>
)rawliteral";
