const fs = require('fs');
const path = require('path');

const files = [
  'docs/chapter-3-isaac-ai-brain/dynamic-gait-transitions.md',
  'docs/chapter-3-isaac-ai-brain/final-documentation-package.md',
  'docs/chapter-3-isaac-ai-brain/walking-gait-integration.md'
];

files.forEach(file => {
  const content = fs.readFileSync(file, 'utf8');

  // Fix corrupted text patterns
  let fixed = content
    // Fix corrupted string interpolation
    .replace(/%\)" symbol\{[^}]*\}/g, '%")  \/\/ SC-003 requirement')
    // Fix improper backticks in code block titles
    .replace(/```(\w+) title="([^"]+)```/g, '```$1 title="$2"')
    // Fix line 386 issue
    .replace(/" symbol\{[^}]*\}\\"/g, '')
    // Escape curly braces in bash scripts that aren't meant to be JSX
    .replace(/(log_pass|log_fail)\s*\(\s*"([^"]*)\$\{([^}]*)\}([^"]*)"\s*\)/g,
      (match, func, before, varName, after) => {
        return `${func} "${before}\${${varName}}${after}"`;
      })
    // Fix escaped HTML entities in bash
    .replace(/\\u003e/g, '>')
    .replace(/\\u0026\\u0026/g, '&&')
    // Fix corrupted bash variable assignments
    .replace(/color_\w+='033\[/g, (match) => match.replace("'033[", "'\\033["))
    // Fix weird characters
    .replace(/만\s*\\\(/g, '');

  fs.writeFileSync(file, fixed, 'utf8');
  console.log(`Fixed ${file}`);
});