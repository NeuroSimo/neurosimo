module.exports = {
    'env': {
        'browser': true,
        'es2021': true,
        'node': true,
    },
    'extends': [
        'eslint:recommended',
        'plugin:react/recommended',
        'plugin:@typescript-eslint/recommended',
        'prettier'
    ],
    'parser': '@typescript-eslint/parser',
    'parserOptions': {
        'ecmaFeatures': {
            'jsx': true
        },
        'ecmaVersion': 'latest',
        'sourceType': 'module'
    },
    'plugins': ['react', 'react-hooks', '@typescript-eslint', 'prettier'],
    'rules': {
        'no-console': ['off'],
        'react/prop-types': ['off'],
        'import/prefer-default-export': ['off'],
        'prefer-destructuring': ['off'],
        'no-alert': ['off'],
        'no-unused-vars': 'off',
        '@typescript-eslint/no-unused-vars': ['warn'],
        '@typescript-eslint/explicit-module-boundary-types': 'off',
        quotes: [2, 'single', 'avoid-escape'],
        '@typescript-eslint/no-explicit-any': ['off'],
        'eol-last': 'error',
        'semi': ['error', 'never'],
        'max-len': [
            'error',
            {
                code: 120,
                tabWidth: 2,
                comments: 120,
                ignoreComments: false,
                ignoreTrailingComments: true,
                ignoreUrls: true,
                ignoreStrings: true,
                ignoreTemplateLiterals: true,
                ignoreRegExpLiterals: true,
            },
        ],
    },
    'settings': {
        'import/resolver': {
            'typescript': {}
        }
    }
}
