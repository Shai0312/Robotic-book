#!/usr/bin/env node

const fs = require('fs');
const path = require('path');

// Configuration
const DOCS_DIR = './docs';
const IGNORE_DIRS = ['.git', 'node_modules', 'build', 'scripts'];

// Validation functions
function validateMarkdownFiles() {
    console.log('🔍 Validating Markdown files...');

    const errors = [];
    const warnings = [];

    // Walk through docs directory
    function walkDir(dir) {
        const files = fs.readdirSync(dir);

        files.forEach(file => {
            const filePath = path.join(dir, file);
            const stat = fs.statSync(filePath);

            if (stat.isDirectory() && !IGNORE_DIRS.includes(file)) {
                walkDir(filePath);
            } else if (file.endsWith('.md')) {
                const content = fs.readFileSync(filePath, 'utf8');

                // Check for common issues
                const fileErrors = [];
                const fileWarnings = [];

                // Check for frontmatter
                if (!content.startsWith('---')) {
                    fileWarnings.push('Missing frontmatter (---)');
                }

                // Check for broken links
                const linkRegex = /\[([^\]]+)\]\(([^)]+)\)/g;
                let match;
                while ((match = linkRegex.exec(content)) !== null) {
                    const link = match[2];
                    if (link.startsWith('./') || link.startsWith('../')) {
                        // Check if relative link exists
                        const linkPath = path.resolve(path.dirname(filePath), link);
                        if (!fs.existsSync(linkPath)) {
                            fileErrors.push(`Broken relative link: ${link} in ${filePath}`);
                        }
                    }
                }

                // Check for missing alt text in images
                const imgRegex = /!\[([^\]]*)\]\(([^)]+)\)/g;
                let imgMatch;
                while ((imgMatch = imgRegex.exec(content)) !== null) {
                    const altText = imgMatch[1];
                    if (!altText || altText.trim() === '') {
                        fileWarnings.push(`Missing alt text for image: ${imgMatch[0]} in ${filePath}`);
                    }
                }

                if (fileErrors.length > 0) {
                    errors.push({ file: filePath, errors: fileErrors });
                }

                if (fileWarnings.length > 0) {
                    warnings.push({ file: filePath, warnings: fileWarnings });
                }
            }
        });
    }

    walkDir(DOCS_DIR);

    return { errors, warnings };
}

function validateURDFFiles() {
    console.log('🔍 Validating URDF files...');

    const urdfFiles = [];
    const errors = [];

    // Find URDF files
    function walkDir(dir) {
        const files = fs.readdirSync(dir);

        files.forEach(file => {
            const filePath = path.join(dir, file);
            const stat = fs.statSync(filePath);

            if (stat.isDirectory() && !IGNORE_DIRS.includes(file)) {
                walkDir(filePath);
            } else if (file.endsWith('.urdf') || file.endsWith('.xacro')) {
                urdfFiles.push(filePath);

                // Basic XML validation
                try {
                    const content = fs.readFileSync(filePath, 'utf8');
                    if (!content.includes('<?xml') && !content.includes('<robot')) {
                        errors.push(`Invalid URDF/Xacro file: ${filePath} - missing XML declaration or robot tag`);
                    }
                } catch (e) {
                    errors.push(`Cannot read URDF file: ${filePath} - ${e.message}`);
                }
            }
        });
    }

    walkDir(DOCS_DIR);

    return { urdfFiles, errors };
}

function validateSidebarReferences() {
    console.log('🔍 Validating sidebar references...');

    const sidebarPath = './sidebars.js';
    if (!fs.existsSync(sidebarPath)) {
        return { errors: [`Sidebar file not found: ${sidebarPath}`] };
    }

    const sidebarContent = fs.readFileSync(sidebarPath, 'utf8');
    const errors = [];

    // Extract doc references from sidebar
    const docRegex = /'([^']+\.(md|mdx))'/g;
    let match;
    const sidebarDocs = [];

    while ((match = docRegex.exec(sidebarContent)) !== null) {
        let docPath = match[1];
        // Convert to file path
        docPath = path.join('./docs', docPath + '.md');

        if (!fs.existsSync(docPath)) {
            errors.push(`Sidebar references non-existent file: ${docPath}`);
        }
    }

    return { errors };
}

function runValidation() {
    console.log('🚀 Starting content validation...\n');

    const markdownResults = validateMarkdownFiles();
    const urdfResults = validateURDFFiles();
    const sidebarResults = validateSidebarReferences();

    // Report results
    console.log('\n📊 Validation Results:');

    if (markdownResults.errors.length > 0) {
        console.log('\n❌ Markdown Errors Found:');
        markdownResults.errors.forEach(result => {
            console.log(`  File: ${result.file}`);
            result.errors.forEach(error => console.log(`    - ${error}`));
        });
    }

    if (markdownResults.warnings.length > 0) {
        console.log('\n⚠️  Markdown Warnings:');
        markdownResults.warnings.forEach(result => {
            console.log(`  File: ${result.file}`);
            result.warnings.forEach(warning => console.log(`    - ${warning}`));
        });
    }

    if (urdfResults.errors.length > 0) {
        console.log('\n❌ URDF Errors Found:');
        urdfResults.errors.forEach(error => console.log(`  - ${error}`));
    }

    if (sidebarResults.errors.length > 0) {
        console.log('\n❌ Sidebar Errors Found:');
        sidebarResults.errors.forEach(error => console.log(`  - ${error}`));
    }

    // Summary
    const totalErrors =
        markdownResults.errors.length +
        urdfResults.errors.length +
        sidebarResults.errors.length;

    const totalWarnings = markdownResults.warnings.length;

    console.log(`\n📈 Summary:`);
    console.log(`  Errors: ${totalErrors}`);
    console.log(`  Warnings: ${totalWarnings}`);

    if (totalErrors === 0) {
        console.log('\n✅ All validations passed!');
        process.exit(0);
    } else {
        console.log('\n❌ Some validations failed. Please fix the errors above.');
        process.exit(1);
    }
}

// Run validation
runValidation();