# ERC25_navigation
# Introduction
This is the official repo for Mind cloud team ERC25 on-stie competition code.

# System requirement
This repo will work on **Ubuntu24** with **ROS2 Jazzy**
- [Ubuntu24 ISO](https://ubuntu.com/download/desktop/thank-you?version=24.04.2&architecture=amd64&lts=true)
- [ROS2 Jazzy installation](https://docs.ros.org/en/jazzy/Installation.html)

Instead of downlaoding docker compose repo is avaliable to use:
[ROS2 Jazzy docker repo](https://github.com/MindCloud-Team/ROS2_Jazzy_docker)

# Git Workflow Guidelines
## Branch Structure

Our repository follows a structured branching model to ensure stability and organize development:

- **`main`**: The last stable version of the system. This branch is protected and only receives updates from `devel` after thorough testing.
- **`devel`**: The active development branch where all feature work is merged after review.
- **`other`**: Individual task branches where developers work on specific task.

```
main (stable) ← devel (integration) ← task branches (features/fixes)
```

## Workflow Process

1. All development work happens in task-specific branches
2. Changes are merged into `devel` via pull requests
3. After testing, `devel` is merged upstream to `main`

## Task Branch Guidelines

- Each task should have its own branch

## Pull Request Process

Pull requests to `devel` will be evaluated based on:
- Code style adherence
- Complete and proper documentation
- Logical correctness
- Test coverage

## Example: Working with Task Branches

### Creating and pushing to a task branch

```bash
# Ensure you're starting from the latest devel
git checkout devel
git pull origin devel

# Create and switch to a new task branch
git checkout -b <your-branch>

# Make your changes, then commit them
git add .
git commit -m "ADD: Login form component and validation"

# Push your branch to remote
git push -u origin <your-branch>
```

### Creating a Pull Request (GitHub)

1. Go to the repository on GitHub
2. Click on "Pull requests" tab
3. Click the "New pull request" button
4. Set "base" branch to `devel`
5. Set "compare" branch to your task branch
6. Click "Create pull request"
7. Add a descriptive title and detailed description
8. Request reviews from team members
9. Submit the pull request

### Standard Commit Types

- **ADD:** Adding new features or functionality
- **FIX:** Bug fixes
- **DOCS:** Documentation changes only
- **STYLE:** Code style changes (formatting, missing semi-colons, etc.)
- **REFACTOR:** Code changes that neither fix bugs nor add features
- **TEST:** Adding or modifying tests
- **CHORE:** Changes to build process, dependencies, etc.
- **PERF:** Performance improvements

### Examples of Good Commit Messages

```
ADD: User authentication module

Implements login, registration, and password reset functionality
```

```
FIX: Correct calculation in tax processing function

Resolves issue #42 where tax calculation was incorrect for certain income ranges
```

```
REFACTOR: Improve error handling in API client
```

```
DOCS: Update installation instructions
```

Remember that good commit messages make it easier to track changes and understand the project history.

# Python Code Style and Documentation Guidelines

## Code Style

We follow PEP 8 guidelines with some additional project-specific conventions. Consistent code style makes our codebase easier to read, maintain, and debug.

### Naming Conventions

- **Classes**: Use `CamelCase` (e.g., `UserProfile`, `DatabaseManager`)
- **Functions/Methods**: Use `snake_case` (e.g., `calculate_total`, `get_user_data`)
- **Variables**: Use `snake_case` (e.g., `user_name`, `total_count`)
- **Constants**: Use `UPPER_SNAKE_CASE` (e.g., `MAX_RETRY_COUNT`, `API_KEY`)
- **Modules**: Use short, `lowercase` names (e.g., `utils.py`, `auth.py`)
- **Private attributes/methods**: Prefix with underscore (e.g., `_internal_method`, `_private_var`)

### Formatting

- Maximum line length: 79 characters
- Add blank lines between functions and classes (2 lines)
- Add blank lines between logical sections of code (1 line)
- Use spaces around operators (`x = 1 + 2`, not `x=1+2`)

## Docstrings

All modules, classes, methods, and functions must include docstrings following the Google style format.

### Module Docstrings

Place at the top of the file, describing the module's purpose:

```python
"""
This module provides authentication functionality for the application.

It includes user registration, login, password reset, and session management.
"""
```

### Class Docstrings

```python
class UserManager:
    """Handles user operations and management.
    
    This class provides methods for creating, retrieving, updating, 
    and deleting user accounts, as well as handling authentication
    and permissions.
    
    Attributes:
        db_connection: Database connection object
        cache_enabled (bool): Whether caching is enabled
    """
```

### Method and Function Docstrings

```python
def calculate_tax(income, tax_rate=0.2):
    """Calculate the tax amount based on income and tax rate.
    
    Args:
        income (float): The total income amount
        tax_rate (float, optional): The tax rate as a decimal. Defaults to 0.2.
        
    Returns:
        float: The calculated tax amount
        
    Raises:
        ValueError: If income is negative
        TypeError: If income or tax_rate is not a number
        
    Examples:
        >>> calculate_tax(50000)
        10000.0
        >>> calculate_tax(50000, 0.3)
        15000.0
    """
```

### Property Docstrings

```python
@property
def full_name(self):
    """str: The user's full name combining first and last name."""
    return f"{self.first_name} {self.last_name}"
```

## Example Class with Complete Documentation

```python
"""
User management module providing CRUD operations for user accounts.

This module handles user data validation, storage, and retrieval.
"""

from datetime import datetime
from typing import Optional, List, Dict, Any


class User:
    """Represents a user in the system.
    
    This class manages user data including personal information,
    authentication details, and user permissions.
    
    Attributes:
        username (str): Unique identifier for the user
        email (str): User's email address
        first_name (str): User's first name
        last_name (str): User's last name
        is_active (bool): Whether the user account is active
        created_at (datetime): When the user was created
    """
    
    def __init__(self, username: str, email: str, 
                 first_name: str = "", last_name: str = ""):
        """Initialize a new User instance.
        
        Args:
            username (str): Unique identifier for the user
            email (str): User's email address
            first_name (str, optional): User's first name. Defaults to empty string.
            last_name (str, optional): User's last name. Defaults to empty string.
        
        Raises:
            ValueError: If username or email is empty
        """
        if not username or not email:
            raise ValueError("Username and email are required")
            
        self.username = username
        self.email = email
        self.first_name = first_name
        self.last_name = last_name
        self.is_active = True
        self.created_at = datetime.now()
        self._login_attempts = 0
    
    @property
    def full_name(self) -> str:
        """str: The user's full name combining first and last name."""
        return f"{self.first_name} {self.last_name}".strip() or "No Name"
    
    def update_profile(self, **kwargs: Any) -> None:
        """Update user profile information.
        
        Args:
            **kwargs: Arbitrary keyword arguments representing
                      user attributes to update.
                      
        Examples:
            >>> user.update_profile(first_name="John", last_name="Doe")
        """
        for key, value in kwargs.items():
            if hasattr(self, key):
                setattr(self, key, value)
    
    def deactivate(self) -> None:
        """Deactivate this user account.
        
        This prevents the user from logging in but preserves their data.
        """
        self.is_active = False
        
    def reactivate(self) -> None:
        """Reactivate a previously deactivated user account."""
        self.is_active = True
        
    def __str__(self) -> str:
        """Return string representation of user.
        
        Returns:
            str: User string with username and full name
        """
        return f"User({self.username}: {self.full_name})"
```

## Code Review Checklist

Before submitting a pull request, ensure your code meets these requirements:

- ✅ Code follows the style guidelines
- ✅ All functions, classes, and methods have appropriate docstrings
- ✅ Type hints are used where appropriate
- ✅ Tests are included for new functionality
- ✅ No unnecessary commented-out code
- ✅ No debugging print statements
- ✅ Error handling is appropriate and specific
- ✅ Variable names are clear and descriptive
