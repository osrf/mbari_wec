document.addEventListener('copy', function(e){
  var text = window.getSelection().toString().replaceAll('$ ', '');
  e.clipboardData.setData('text/plain', text);
  e.preventDefault();
});
