const { Dolos, File } = require("@dodona/dolos-lib");

async function analyzeTexts(text1, text2, langExtension) {
  let file1 = new File(`code1.${langExtension}`, text1);
  let file2 = new File(`code2.${langExtension}`, text2);
  let files = [file1, file2]
  
  const dolos = new Dolos();
  const report = await dolos.analyze(files, "text1 vs text2");
  return report;
}

async function compareCode(code1, code2) {  
    const report = await analyzeTexts(code1, code2, "py");
    
    var result = [];
    for (const pair of report.allPairs()) {
     for (const fragment of pair.buildFragments()) {
        const left = fragment.leftSelection;
        const right = fragment.rightSelection;
        result.push([left.startRow, left.endRow, right.startRow, right.endRow])
     }
     return result;
    }
}

module.exports = { compareCode }