function sendRequest() {
	var http = new XMLHttpRequest();
	http.open("POST", "/get_current_job/", true);
	http.onreadystatechange = function () { handleResponse(http) };
	http.send(null);
}



function handleResponse(http) {
	var response;
	if (http.readyState == 4) {
		response = http.responseText;
		$("#containerContent").fadeIn(300);
		document.getElementById("containerContent").innerHTML = response
		setTimeout("sendRequest()", 1000)
	}
}