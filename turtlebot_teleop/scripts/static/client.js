function sendRequest() {
	var http = new XMLHttpRequest();
	http.open("GET", "/get_current_job/", true);
	http.onreadystatechange = function () { handleResponse(http) };
	http.send(null);
}



function handleResponse(http) {
	var response;
	if (http.readyState == 4) {
		response = http.responseText;
		$("#mainContent").fadeIn(300);
		document.getElementById("mainContent").innerHTML = response
		setTimeout("sendRequest()", 1000)
	}
}