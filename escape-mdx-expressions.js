const fs = require('fs');

function escapeMDXExpressions(filename) {
  let content = fs.readFileSync(filename, 'utf8');

  // Split into lines to track code blocks
  let lines = content.split('\n');
  let inPythonBlock = false;
  let inAnyBlock = false;
  let modified = false;

  for (let i = 0; i < lines.length; i++) {
    const line = lines[i];

    // Check for code block start
    if (line.startsWith('```')) {
      if (line.includes('python')) {
        inPythonBlock = true;
      } else {
        inAnyBlock = !inAnyBlock;
      }
    } else if (line.startsWith('```') && inPythonBlock) {
      // End of python block
      inPythonBlock = false;
    }

    // Skip if we're in a Python code block
    if (!inPythonBlock && !inAnyBlock) {
      // Look for f-string patterns with curly braces
      if (line.includes('f"') && line.includes('{') && line.includes('}')) {
        // Check if it's an f-string outside of code block
        const match = line.match(/^([^`]*f"[^"]*{[^}]*}[^"]*"[^`]*)$/);
        if (match) {
          // Escape the curly braces by adding backticks
          lines[i] = '\n```python\n' + line + '\n```\n';
          modified = true;
        }
      }
    }
  }

  if (modified) {
    fs.writeFileSync(filename, lines.join('\n'), 'utf8');
    console.log(`Fixed ${filename}`);
  } else {
    console.log(`No changes needed for ${filename}`);
  }
}

// Process all problematic files
const files = [
  'docs/chapter-3-isaac-ai-brain/final-documentation-package.md',
];

files.forEach(file => {
  if (fs.existsSync(file)) {
    escapeMDXExpressions(file);
  }
});