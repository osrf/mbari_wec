$(document).ready(function() {
  // https://clipboardjs.com/
  var selectors = document.querySelectorAll('pre code');
  var copyButton = '<div class="clipboard"><span class="btn btn-neutral btn-clipboard" title="Copy to clipboard">Copy</span></div>';
  Array.prototype.forEach.call(selectors, function(selector){
    selector.insertAdjacentHTML('beforebegin', copyButton);
  });
  var clipboard = new ClipboardJS('.btn-clipboard', {
    target: function (trigger) {
      return trigger.parentNode.nextElementSibling;
    }
  });

  clipboard.on('success', function (e) {
    e.clearSelection();

    // https://atomiks.github.io/tippyjs/v6/all-props/
    var tippyInstance = tippy(
      e.trigger,
      {
        content: 'Copied',
        showOnCreate: true,
        trigger: 'manual',
      },
    );
    setTimeout(function() { tippyInstance.hide(); }, 1000);
  });
});
