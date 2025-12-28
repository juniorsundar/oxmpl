import { execSync } from 'node:child_process';
import fs from 'node:fs';
import path from 'node:path';

const rootDir = path.resolve(process.cwd());
const pkgBundlerDir = path.join(rootDir, 'pkg-bundler');

console.log('Building WASM package...');
try {
  execSync('npm run build', { stdio: 'inherit' });
} catch (error) {
  console.error('Build failed:', error);
  process.exit(1);
}

console.log('Copying and adjusting index.js...');
const indexSrc = path.join(rootDir, 'index.js');
const indexDest = path.join(pkgBundlerDir, 'index.js');

let indexContent = fs.readFileSync(indexSrc, 'utf8');
// Adjust import path: ./pkg-bundler/oxmpl_js.js -> ./oxmpl_js.js
indexContent = indexContent.replace(
  /from ['"]\.\/pkg-bundler\/oxmpl_js\.js['"]/,
  "from './oxmpl_js.js'"
);
fs.writeFileSync(indexDest, indexContent);

console.log('Copying and adjusting index.d.ts...');
const dtsSrc = path.join(rootDir, 'index.d.ts');
const dtsDest = path.join(pkgBundlerDir, 'index.d.ts');

let dtsContent = fs.readFileSync(dtsSrc, 'utf8');
// Adjust import path: ./pkg-bundler/oxmpl_js -> ./oxmpl_js
dtsContent = dtsContent.replace(/from ['"]\.\/pkg-bundler\/oxmpl_js['"]/, "from './oxmpl_js'");
fs.writeFileSync(dtsDest, dtsContent);

console.log('Updating pkg-bundler/package.json...');
const pkgJsonPath = path.join(pkgBundlerDir, 'package.json');
const pkgJson = JSON.parse(fs.readFileSync(pkgJsonPath, 'utf8'));

// Set main entry point to the wrapper
pkgJson.main = 'index.js';
// Set types entry point to the wrapper definition
pkgJson.types = 'index.d.ts';
// Add type module just in case (wasm-pack adds it usually)
pkgJson.type = 'module';

// Ensure index.d.ts is included in the package files
if (pkgJson.files && !pkgJson.files.includes('index.d.ts')) {
  pkgJson.files.push('index.d.ts');
}

fs.writeFileSync(pkgJsonPath, JSON.stringify(pkgJson, null, 2));

console.log('Package preparation complete.');
