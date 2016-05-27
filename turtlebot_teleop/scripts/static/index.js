var itemPickup;

function loginFadeIn(result) {
	document.getElementById("mainContent").innerHTML = result;
	$("#mainContent").fadeIn(500);
	$("#top-bar").fadeIn(500);
	$("#top-bar").addClass("form-success")
	setTimeout(function () { $("#top-bar").removeClass("main move-shit-up form-success") }, 1500);
	reloadJs("/static/index.js");
}

function pageFadeIn(result) {
	document.getElementById("mainContent").innerHTML = result;
	$("#mainContent").fadeIn(500);
	reloadJs("/static/index.js");
	sendRequest();
}

$("#loginForm").submit(function (event) {
	event.preventDefault();
	$.ajax({
		type: $("#loginForm").attr('method'),
		url: $("#loginForm").attr('action'),
		data: $("#loginForm").serialize(),
		success: function (result) {
			console.log(result);

			if (result == "failure") {
				document.getElementById("LoginMsg").innerHTML = "Couldn't log you in. Please try again";
			} else {
				$("#mainContent").fadeOut(500);
				$("#top-bar").addClass("move-shit-up")
				setTimeout(function () { loginFadeIn(result); }, 250);

			}
		},
		error: function () {
			document.getElementById("LoginMsg").innerHTML = "Couldn't log you in. Please try again";
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
			$("#mainContent").fadeOut(300);
			setTimeout(function() { pageFadeIn(result); }, 250);
		},
		error: function () {
			// Do nothing
		}
	});
});

$("#loadingForm").submit(function (event) {
	event.preventDefault();
	$.ajax({
		type: $("#loadingForm").attr('method'),
		url: $("#loadingForm").attr('action'),
		data: $("#loadingForm").serialize(),
		success: function (result) {
			console.log(result);

			if (result == "failure") {
				document.getElementById("LoginMsg").innerHTML = "Couldn't log you in. Please try again";
			} else {
				$("#mainContent").fadeOut(300);
				setTimeout(function() { pageFadeIn(result); }, 250);
			}

		},
		error: function () {
			document.getElementById("LoginMsg").innerHTML = "Couldn't log you in. Please try again";
			// Do nothing
		}
	});
});

$("#completeForm").submit(function (event) {
	event.preventDefault();
	$.ajax({
		type: $("#completeForm").attr('method'),
		url: $("#completeForm").attr('action'),
		data: $("#completeForm").serialize(),
		success: function (result) {
			console.log(result);

			if (result == "failure") {
				document.getElementById("LoginMsg").innerHTML = "Couldn't log you in. Please try again";
			} else {
				$("#mainContent").fadeOut(300);
				setTimeout(function() { pageFadeIn(result); }, 250);
				document.getElementById("mainContent").innerHTML = result;
				clearTimeout(itemPickup);
			}
		},
		error: function () {
			document.getElementById("LoginMsg").innerHTML = "Couldn't log you in. Please try again";
			// Do nothing
		}
	});
});

$(function() {
	$("#logoutText").click(function (e) {
		e.preventDefault(); // if desired...
		document.cookie = "userID=; expires=Thu, 01 Jan 1970 00:00:00 UTC";
		location.reload(true);
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
		if (response != "Your item is ready at its destination!") {
			// document.getElementById("complete").style.visibility = "hidden";
			// document.getElementById("spin").style.visibility = "visible";
			document.getElementById("waittime").innerHTML = response
			setTimeout("sendRequest()", 1000);
		}
		else {
			$(function() {
				$("#loading_content").fadeOut(500);
				setTimeout("fadeInComplete()", 500);
				// $("#complete").fadeIn(500);
				// $('#complete').addClass('form-success');
				// setTimeout(function() {$('#wrapper').addClass('form-success');}, 500);
				// document.getElementById("complete").style.visibility = "visible";
				// document.getElementById("spin").style.visibility = "hidden";
				itemPickup = setTimeout("endRequest()", 3100);
			});
		}
	}
}

function fadeInComplete() {
	$("#complete").fadeIn(500);
	$('#complete').addClass('form-success');
}

function endRequest() {
	// Make Ajax request to grab page from server
	var http = new XMLHttpRequest();
	var username = getCookie("userID");
	http.open("GET", "/end_request" + username, true);
	http.onreadystatechange = function () { handleEndResponse(http) };
	http.send(null);
}

function handleEndResponse(http) {
	var response;
	if (http.readyState == 4) {
		response = http.responseText;
		$("#mainContent").fadeOut(300);
		setTimeout(function () { pageFadeIn(result); }, 250);
		document.getElementById("mainContent").innerHTML = response;
	}
}

function reloadJs(src) {
    src = $('script[src$="' + src + '"]').attr("src");
    $('script[src$="' + src + '"]').remove();
    $('<script/>').attr('src', src).appendTo('body');
}