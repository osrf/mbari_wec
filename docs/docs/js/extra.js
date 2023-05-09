
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

document.addEventListener('copy', function(e){
  var text = window.getSelection().toString()
  var text_ = text;
  var indices = getIndicesOf('$ ', text);
  console.log(indices);
  var sel = window.getSelection();
  var rng_all = sel.getRangeAt(0);
  console.log([rng_all.startOffset, rng_all.endOffset]);
  var start_node = rng_all.startContainer;
  var end_node = rng_all.endContainer;
  sel.removeAllRanges();
  if (!rng_all.collapsed && (indices.length > 0)) {
    text_ = text.replaceAll('$ ', '');
    let idx = 0;
    var rng = new Range();
    do {
      rng.setStart(start_node, rng_all.startOffset + indices[idx] + 2);
      if (idx + 1 < indices.length) {
        rng.setEnd(start_node, indices[idx + 1]);
      } else {
        rng.setEnd(end_node, rng_all.endOffset);
      }
      sel.addRange(rng);
      idx = idx + 1;
    } while(idx < indices.length - 1);
  }
  e.clipboardData.setData('text/plain', text_);
  e.preventDefault();
});
