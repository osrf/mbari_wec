// https://stackoverflow.com/a/3410557/9686600
function getIndicesOf(searchStr, str, caseSensitive) {
    var searchStrLen = searchStr.length;
    if (searchStrLen == 0) {
        return [];
    }
    var startIndex = 0, index, indices = [];
    if (!caseSensitive) {
        str = str.toLowerCase();
        searchStr = searchStr.toLowerCase();
    }
    while ((index = str.indexOf(searchStr, startIndex)) > -1) {
        indices.push(index);
        startIndex = index + searchStrLen;
    }
    return indices;
}

// https://code-boxx.com/detect-browser-with-javascript/
function isFF() {
  var isFirefox = typeof InstallTrigger !== 'undefined';  // Warning: InstallTrigger deprecated and may be removed
  return isFirefox;
}

// https://stackoverflow.com/questions/21229067/firefox-triple-click-selection-returns-incorrect-start-and-end-offsets
function adjustRange(range) {
  if(isFF()) {
    range = range.cloneRange();
    if (range.startContainer.nodeType != 3) {
        var nodeAfterStart = range.startContainer.childNodes[range.startOffset];
        if (nodeAfterStart && nodeAfterStart.nodeType == 3) {
            range.setStart(nodeAfterStart, 0);
        }
    }
    if (range.endContainer.nodeType != 3 && range.endOffset >= 1) {
        var nodeBeforeEnd = range.endContainer.childNodes[range.endOffset - 1];
        if (nodeBeforeEnd && nodeBeforeEnd.nodeType == 3) {
            range.setEnd(nodeBeforeEnd, nodeBeforeEnd.data.length);
        }
    }
  }
  return range;
}

// https://stackoverflow.com/a/56592247/9686600
function getSelectionText() {
  if (window.getSelection) {
    try {
      var activeElement = document.activeElement;
      if (activeElement && activeElement.value) {
        // firefox bug https://bugzilla.mozilla.org/show_bug.cgi?id=85686
        return activeElement.value.substring(activeElement.selectionStart, activeElement.selectionEnd);
      } else {
        return window.getSelection().toString();
      }
    } catch (e) {
    }
  } else if (document.selection && document.selection.type != "Control") {
    // For IE
    return document.selection.createRange().text;
  }
}

document.addEventListener('copy', function(e){
  var text = getSelectionText();
  console.log(text);
  var text_ = text;
  var indices = getIndicesOf('$ ', text);
  console.log('indices: ' + indices);
  var sel = window.getSelection();
  var rng_all = adjustRange(sel.getRangeAt(0));
  console.log('rng_all start end:', [rng_all.startOffset, rng_all.endOffset]);
  var start_node = rng_all.startContainer;
  var end_node = rng_all.endContainer;
  if (indices.length > 0) {
    text_ = text.replaceAll('$ ', '');
    console.log(text_);
    if(!rng_all.collapsed) {
      sel.removeAllRanges();
      let idx = 0;
      var rng = new Range();
      do {
        console.log('idx:', idx);
        console.log('start offset', rng_all.startOffset + indices[idx] + 2);
        rng.setStart(start_node, rng_all.startOffset + indices[idx] + 2);
        if (idx + 1 < indices.length) {
          if(isFF()) {
            console.log('Firefox detected!');
            rng.setEnd(start_node, rng_all.startOffset + indices[idx + 1]-2);
          } else {
            rng.setEnd(start_node, rng_all.startOffset + indices[idx + 1]);
          }
        } else {
          rng.setEnd(end_node, rng_all.endOffset);
        }
        sel.addRange(rng);
        idx = idx + 1;
      } while(idx < indices.length - 1);
    }
  }
  e.clipboardData.setData('text/plain', text_);
  e.preventDefault();
});
