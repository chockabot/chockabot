$("#loginForm").submit(function (event) {
	event.preventDefault();
	$.ajax({
		type: $("#loginForm").attr('method'),
		url: $("#loginForm").attr('action'),
		data: $("#loginForm").serialize(),
		success: function (result) {
			console.log(result);
			
				// $('#mainContent').fadeOut(300);
				// $('.wrapper').addClass('form-success');
				// $("#mainContent").html = result;
				document.getElementById("mainContent").innerHTML = result;
				$("#mainContent").fadeIn(300);
				$("#top-bar").fadeIn(500);
				// $('#mainContent').html = result;
				
				// window.location.href =  "{{ url_for('user', name=name) }}"
				
			// }
			// else {
			// 	$("LoginMsg").text("Couldn't log you in!")
			// 	alert("Failed to login")
			// }
		},
		error: function() {
			// Do nothing
		}
	});
});

$("#requestForm").submit(function (event) {
	event.preventDefault();
	$.ajax({
		type: $("#requestForm").attr('method'),
		url: $("#requestForm").attr('action'),
		data: $("#requestForm").serialize(),
		success: function (result) {
			console.log(result);
			
				// $('#mainContent').fadeOut(300);
				// $('.wrapper').addClass('form-success');
				// $("#mainContent").html = result;
				document.getElementById("mainContent").innerHTML = result;
				$("#mainContent").fadeIn(300);
				sendRequest();
				// $("#top-bar").fadeIn(500);
				// $('#mainContent').html = result;
				
				// window.location.href =  "{{ url_for('user', name=name) }}"
				
			// }
			// else {
			// 	$("LoginMsg").text("Couldn't log you in!")
			// 	alert("Failed to login")
			// }
		},
		error: function() {
			// Do nothing
		}
	});
});

function getCookie(cname) {
    var name = cname + "=";
    var ca = document.cookie.split(';');
    for(var i = 0; i <ca.length; i++) {
        var c = ca[i];
        while (c.charAt(0)==' ') {
            c = c.substring(1);
         }
        if (c.indexOf(name) == 0) {
            return c.substring(name.length,c.length);
         }
    }
    return "";
} 

function sendRequest() {
	var http = new XMLHttpRequest();
	var username = getCookie("userID");
	http.open("GET", "/show_status/" + username, true);
	http.onreadystatechange = function () { handleResponse(http) };
	http.send(null);
}

function handleResponse(http) {
	var response;
	if (http.readyState == 4) {
		response = http.responseText;
		document.getElementById("waittime").innerHTML = response
		if (response != "Your item is ready at its destination!") {
			document.getElementById("complete").style.visibility = "hidden";
			document.getElementById("spin").style.visibility = "visible";
			setTimeout("sendRequest()", 1000);
		}
		else {
			document.getElementById("complete").style.visibility = "visible";
			document.getElementById("spin").style.visibility = "hidden";
		}
	}
}